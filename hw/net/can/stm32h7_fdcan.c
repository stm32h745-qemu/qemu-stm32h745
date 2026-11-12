/*
 * STM32H7 FDCAN (Bosch M_CAN 3.x) behavioral model. Enough of the controller for Zephyr's can_mcan/can_stm32h7
 * drivers: message RAM elements, TX buffers with the TX event FIFO, RX FIFO 0
 * and 1, standard and extended acceptance filters (range, dual, classic),
 * global filter, interrupt lines. Classic CAN frames only (no CAN FD data
 * phase, no bit timing, no bus errors).
 *
 * STM32G4 variant ("g4" property, RM0440): the same M_CAN core with the
 * register map STM32 gave it on the G4 (some registers moved or absent, the
 * IR/IE/ILS bits compacted) and a fixed message RAM layout per instance
 * (28 standard filters, 8 extended filters, 3+3 RX FIFO elements, 3 TX event
 * elements, 3 TX buffers, 64 byte data fields; 0x350 bytes at "msg-ram-offset"
 * in the shared SRAMCAN).
 *
 * The bus is a chardev carrying SLCAN text lines (t<iii><l><data>\r and
 * T<iiiiiiii><l><data>\r), so firmware running on the model can share a CAN
 * bus with other emulated nodes or host-side models through a socket hub.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/net/stm32h7_fdcan.h"
#include "exec/memory.h"

/* Register offsets */
#define CREL    0x00
#define ENDN    0x04
#define CCCR    0x18
#define PSR     0x44
#define IR      0x50
#define IE      0x54
#define ILS     0x58
#define ILE     0x5C
#define GFC     0x80
#define SIDFC   0x84
#define XIDFC   0x88
#define XIDAM   0x90
#define RXF0C   0xA0
#define RXF0S   0xA4
#define RXF0A   0xA8
#define RXBC    0xAC
#define RXF1C   0xB0
#define RXF1S   0xB4
#define RXF1A   0xB8
#define RXESC   0xBC
#define TXBC    0xC0
#define TXFQS   0xC4
#define TXESC   0xC8
#define TXBRP   0xCC
#define TXBAR   0xD0
#define TXBCR   0xD4
#define TXBTO   0xD8
#define TXBCF   0xDC
#define TXBTIE  0xE0
#define TXBCIE  0xE4
#define TXEFC   0xF0
#define TXEFS   0xF4
#define TXEFA   0xF8

#define IR_RF0N BIT(0)
#define IR_RF0L BIT(3)
#define IR_RF1N BIT(4)
#define IR_RF1L BIT(7)
#define IR_TC   BIT(9)
#define IR_TEFN BIT(12)
#define IR_TEFL BIT(15)
#define CCCR_INIT BIT(0)
#define ILE_EINT0 BIT(0)
#define ILE_EINT1 BIT(1)

#define R(s, off) ((s)->regs[(off) / 4])

static const int data_size_bytes[8] = { 8, 12, 16, 20, 24, 32, 48, 64 };

static uint32_t *ram(Stm32h7FdcanState *s, uint32_t byte_off)
{
    uint8_t *base = memory_region_get_ram_ptr(s->msg_ram);
    uint64_t size = memory_region_size(s->msg_ram);

    if (s->g4) {
        byte_off += s->ram_offset;
    }

    if (byte_off + 4 > size) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-fdcan: message RAM offset 0x%x out of range\n",
                      byte_off);
        return NULL;
    }
    return (uint32_t *)(base + byte_off);
}

static int elem_words(uint32_t dscode)
{
    return 2 + data_size_bytes[dscode & 7] / 4;
}

/* ---- message RAM geometry: from registers (H7), fixed (G4) ---- */

#define G4_SID_BASE   0x000
#define G4_XID_BASE   0x070
#define G4_RXF0_BASE  0x0B0
#define G4_RXF1_BASE  0x188
#define G4_TXEF_BASE  0x260
#define G4_TXB_BASE   0x278
#define G4_FIFO_ELEMS 3
#define G4_ELEM_WORDS 18

static uint32_t rx_base(Stm32h7FdcanState *s, int fifo)
{
    return s->g4 ? (fifo ? G4_RXF1_BASE : G4_RXF0_BASE) :
                   (R(s, fifo ? RXF1C : RXF0C) & 0xFFFC);
}

static uint32_t rx_size(Stm32h7FdcanState *s, int fifo)
{
    return s->g4 ? G4_FIFO_ELEMS : (R(s, fifo ? RXF1C : RXF0C) >> 16) & 0x7F;
}

static uint32_t rx_words(Stm32h7FdcanState *s, int fifo)
{
    return s->g4 ? G4_ELEM_WORDS : elem_words(fifo ? (R(s, RXESC) >> 4) : R(s, RXESC));
}

static uint32_t txef_base(Stm32h7FdcanState *s)
{
    return s->g4 ? G4_TXEF_BASE : (R(s, TXEFC) & 0xFFFC);
}

static uint32_t txef_size(Stm32h7FdcanState *s)
{
    return s->g4 ? G4_FIFO_ELEMS : (R(s, TXEFC) >> 16) & 0x3F;
}

static uint32_t txb_base(Stm32h7FdcanState *s)
{
    return s->g4 ? G4_TXB_BASE : (R(s, TXBC) & 0xFFFC);
}

static uint32_t txb_words(Stm32h7FdcanState *s)
{
    return s->g4 ? G4_ELEM_WORDS : elem_words(R(s, TXESC));
}

static void update_irq(Stm32h7FdcanState *s)
{
    uint32_t pending = R(s, IR) & R(s, IE);
    uint32_t line1 = pending & R(s, ILS);
    uint32_t line0 = pending & ~R(s, ILS);

    qemu_set_irq(s->irq[0], line0 && (R(s, ILE) & ILE_EINT0));
    qemu_set_irq(s->irq[1], line1 && (R(s, ILE) & ILE_EINT1));
}

/* ---- transmit ---- */

static void slcan_send(Stm32h7FdcanState *s, uint32_t id, bool xtd, bool rtr, int dlc,
                       const uint8_t *data)
{
    char out[40];
    int n;

    if (rtr) {
        n = xtd ? snprintf(out, sizeof(out), "R%08X%X", id, dlc) :
                  snprintf(out, sizeof(out), "r%03X%X", id, dlc);
    } else {
        n = xtd ? snprintf(out, sizeof(out), "T%08X%X", id, dlc) :
                  snprintf(out, sizeof(out), "t%03X%X", id, dlc);
        for (int i = 0; i < dlc && i < 8; i++) {
            n += snprintf(out + n, sizeof(out) - n, "%02X", data[i]);
        }
    }
    out[n++] = '\r';
    qemu_chr_fe_write_all(&s->chr, (uint8_t *)out, n);
    s->tx_frames++;
}

static void tx_event(Stm32h7FdcanState *s, uint32_t t0, uint32_t t1)
{
    uint32_t efs = R(s, TXEFS);
    uint32_t size = txef_size(s);
    uint32_t fill = efs & 0x3F;
    uint32_t get = (efs >> 8) & 0x1F;
    uint32_t put = (efs >> 16) & 0x1F;
    uint32_t *e;

    if (size == 0) {
        return;
    }
    if (fill >= size) {
        R(s, IR) |= IR_TEFL;
        return;
    }
    e = ram(s, txef_base(s) + put * 8);
    if (e) {
        e[0] = t0;
        e[1] = (t1 & 0xFF0F0000) | (1u << 22);  /* MM, DLC, EDL/BRS, ET = 01 */
    }
    put = (put + 1) % size;
    fill++;
    R(s, TXEFS) = (efs & ~0x001F1F3F) | fill | (get << 8) | (put << 16) |
                  ((fill == size) ? BIT(25) : 0);
    R(s, IR) |= IR_TEFN;
}

static void transmit(Stm32h7FdcanState *s, uint32_t requests)
{
    uint32_t words = txb_words(s);

    for (int i = 0; i < 32; i++) {
        uint32_t *e;
        uint32_t t0, t1, id;
        bool xtd, rtr;
        int dlc;
        uint8_t data[8];

        if (!(requests & BIT(i))) {
            continue;
        }
        e = ram(s, txb_base(s) + i * words * 4);
        if (!e) {
            continue;
        }
        t0 = e[0];
        t1 = e[1];
        xtd = t0 & BIT(30);
        rtr = t0 & BIT(29);
        id = xtd ? (t0 & 0x1FFFFFFF) : ((t0 >> 18) & 0x7FF);
        dlc = MIN((t1 >> 16) & 0xF, 8);
        memcpy(data, &e[2], 8);
        slcan_send(s, id, xtd, rtr, dlc, data);

        R(s, TXBRP) &= ~BIT(i);
        R(s, TXBTO) |= BIT(i);
        if (R(s, TXBTIE) & BIT(i)) {
            R(s, IR) |= IR_TC;
        }
        if (t1 & BIT(23)) {         /* EFC: store a TX event */
            tx_event(s, t0, t1);
        }
    }
    update_irq(s);
}

static uint32_t txfqs(Stm32h7FdcanState *s)
{
    uint32_t txbc = R(s, TXBC);
    uint32_t ndtb = s->g4 ? 0 : (txbc >> 16) & 0x3F;
    uint32_t tfqs = s->g4 ? G4_FIFO_ELEMS : (txbc >> 24) & 0x3F;
    uint32_t total = MIN(ndtb + tfqs, 32);
    uint32_t free = 0, put = 0;
    bool found = false;

    for (uint32_t i = ndtb; i < total; i++) {
        if (!(R(s, TXBRP) & BIT(i))) {
            free++;
            if (!found) {
                put = i;
                found = true;
            }
        }
    }
    return free | (put << 16) | (free == 0 ? BIT(21) : 0);
}

/* ---- receive ---- */

/* Returns 0: reject, 1: FIFO 0, 2: FIFO 1; *fidx = filter index or -1 */
static int filter(Stm32h7FdcanState *s, uint32_t id, bool xtd, int *fidx)
{
    uint32_t gfc = R(s, GFC);
    uint32_t fc = R(s, xtd ? XIDFC : SIDFC);
    uint32_t count = s->g4 ? (xtd ? (gfc >> 24) & 0xF : (gfc >> 16) & 0x1F) :
                             (fc >> 16) & (xtd ? 0x7F : 0xFF);
    uint32_t fbase = s->g4 ? (xtd ? G4_XID_BASE : G4_SID_BASE) : (fc & 0xFFFC);
    uint32_t mask = xtd ? (R(s, XIDAM) & 0x1FFFFFFF) : 0x7FF;
    uint32_t mid = id & mask;
    uint32_t nonmatch;

    for (uint32_t i = 0; i < count; i++) {
        uint32_t *e = ram(s, fbase + i * (xtd ? 8 : 4));
        uint32_t type, cfg, id1, id2;
        bool match = false;

        if (!e) {
            break;
        }
        if (xtd) {
            cfg = (e[0] >> 29) & 7;
            id1 = e[0] & 0x1FFFFFFF;
            type = (e[1] >> 30) & 3;
            id2 = e[1] & 0x1FFFFFFF;
        } else {
            type = (e[0] >> 30) & 3;
            cfg = (e[0] >> 27) & 7;
            id1 = (e[0] >> 16) & 0x7FF;
            id2 = e[0] & 0x7FF;
        }
        switch (type) {
        case 0: match = mid >= id1 && mid <= id2; break;              /* range */
        case 1: match = mid == id1 || mid == id2; break;              /* dual */
        case 2: match = (mid & id2) == (id1 & id2); break;            /* classic */
        default: break;
        }
        if (!match || cfg == 0) {
            continue;
        }
        *fidx = i;
        switch (cfg) {
        case 1: case 5: return 1;   /* FIFO 0 (5: with priority) */
        case 2: case 6: return 2;   /* FIFO 1 */
        case 3: return 0;           /* reject */
        default: return 1;          /* priority only / buffer: treat as FIFO 0 */
        }
    }
    *fidx = -1;
    nonmatch = xtd ? (gfc >> 2) & 3 : (gfc >> 4) & 3;    /* ANFE / ANFS */
    return nonmatch == 0 ? 1 : nonmatch == 1 ? 2 : 0;
}

static void store_rx(Stm32h7FdcanState *s, int fifo, uint32_t id, bool xtd, bool rtr,
                     int dlc, const uint8_t *data, int fidx)
{
    hwaddr st = fifo ? RXF1S : RXF0S;
    uint32_t size = rx_size(s, fifo);
    uint32_t status = R(s, st);
    uint32_t fill = status & 0x7F;
    uint32_t get = (status >> 8) & 0x3F;
    uint32_t put = (status >> 16) & 0x3F;
    uint32_t words = rx_words(s, fifo);
    uint32_t *e;

    if (size == 0 || fill >= size) {
        R(s, IR) |= fifo ? IR_RF1L : IR_RF0L;
        R(s, st) |= BIT(25);                                /* RFnL: message lost */
        s->rx_dropped++;
        return;
    }
    e = ram(s, rx_base(s, fifo) + put * words * 4);
    if (!e) {
        return;
    }
    e[0] = (xtd ? (BIT(30) | (id & 0x1FFFFFFF)) : ((id & 0x7FF) << 18)) | (rtr ? BIT(29) : 0);
    e[1] = (dlc << 16) | (fidx < 0 ? BIT(31) : ((uint32_t)fidx << 24));
    memset(&e[2], 0, (words - 2) * 4);
    memcpy(&e[2], data, dlc);
    put = (put + 1) % size;
    fill++;
    R(s, st) = (status & ~0x003F3F7F & ~BIT(24)) | fill | (get << 8) | (put << 16) |
               ((fill == size) ? BIT(24) : 0);
    R(s, IR) |= fifo ? IR_RF1N : IR_RF0N;
    s->rx_frames++;
}

static int hexval(char c)
{
    return g_ascii_isxdigit(c) ? g_ascii_xdigit_value(c) : -1;
}

static void receive_line(Stm32h7FdcanState *s, const char *l, uint32_t len)
{
    bool xtd, rtr;
    uint32_t id = 0, idlen;
    int dlc, fifo, fidx;
    uint8_t data[8] = { 0 };

    if (len < 1 || !strchr("tTrR", l[0])) {
        return;
    }
    xtd = l[0] == 'T' || l[0] == 'R';
    rtr = l[0] == 'r' || l[0] == 'R';
    idlen = xtd ? 8 : 3;
    if (len < 1 + idlen + 1) {
        return;
    }
    for (uint32_t i = 0; i < idlen; i++) {
        int v = hexval(l[1 + i]);
        if (v < 0) {
            return;
        }
        id = (id << 4) | v;
    }
    dlc = hexval(l[1 + idlen]);
    if (dlc < 0 || dlc > 8 || (!rtr && len != 1 + idlen + 1 + 2 * (uint32_t)dlc)) {
        return;
    }
    for (int i = 0; !rtr && i < dlc; i++) {
        int hi = hexval(l[2 + idlen + 2 * i]), lo = hexval(l[3 + idlen + 2 * i]);
        if (hi < 0 || lo < 0) {
            return;
        }
        data[i] = (hi << 4) | lo;
    }
    if (R(s, CCCR) & CCCR_INIT) {
        return;                         /* not participating while in init */
    }
    fifo = filter(s, id, xtd, &fidx);
    if (fifo) {
        store_rx(s, fifo - 1, id, xtd, rtr, dlc, data, fidx);
        update_irq(s);
    }
}

static void rx_deliver(void *opaque)
{
    Stm32h7FdcanState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    while (s->rxq_count && now >= s->rx_next_ns) {
        uint32_t i = s->rxq_head;

        s->rxq_head = (s->rxq_head + 1) % 64;
        s->rxq_count--;
        receive_line(s, s->rxq[i], s->rxq_len[i]);
        s->rx_next_ns = now + s->rx_frame_ns;
    }
    if (s->rxq_count) {
        timer_mod(s->rx_timer, s->rx_next_ns);
    }
}

static void rx_line(Stm32h7FdcanState *s, const char *l, uint32_t len)
{
    if (!s->rx_frame_ns) {
        receive_line(s, l, len);
        return;
    }
    if (s->rxq_count == 64) {
        s->rx_dropped++;                /* the bus model's own queue: never on a real bus */
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-fdcan: RX pacing queue full, frame dropped\n");
        return;
    }
    {
        uint32_t i = (s->rxq_head + s->rxq_count) % 64;

        memcpy(s->rxq[i], l, len);
        s->rxq_len[i] = len;
        s->rxq_count++;
    }
    rx_deliver(s);
}

static int chr_can_receive(void *opaque)
{
    return 64;
}

static void chr_receive(void *opaque, const uint8_t *buf, int size)
{
    Stm32h7FdcanState *s = opaque;

    for (int i = 0; i < size; i++) {
        char c = buf[i];
        if (c == '\r' || c == '\n') {
            rx_line(s, s->line, s->line_len);
            s->line_len = 0;
        } else if (s->line_len < sizeof(s->line)) {
            s->line[s->line_len++] = c;
        } else {
            s->line_len = 0;
        }
    }
}

/* ---- STM32G4 register map -> M_CAN offsets and IR/IE/ILS bit layout ---- */

#define G4_CKDIV 0x100

static int g4_reg(hwaddr off)
{
    switch (off) {
    case 0x80: return GFC;
    case 0x84: return XIDAM;
    case 0x90: return RXF0S;
    case 0x94: return RXF0A;
    case 0x98: return RXF1S;
    case 0x9C: return RXF1A;
    case 0xC0: return TXBC;
    case 0xC4: return TXFQS;
    case 0xC8: return TXBRP;
    case 0xCC: return TXBAR;
    case 0xD0: return TXBCR;
    case 0xD4: return TXBTO;
    case 0xD8: return TXBCF;
    case 0xDC: return TXBTIE;
    case 0xE0: return TXBCIE;
    case 0xE4: return TXEFS;
    case 0xE8: return TXEFA;
    case 0x88:                          /* HPMS */
        return -1;
    }
    return off < 0x80 ? (int)off : -1;
}

/* IR/IE: STM32G4 bit layout <-> M_CAN bit layout */
static uint32_t ir_g4_to_mcan(uint32_t b)
{
    return ((b & 0xFF0000u) << 6) | ((b & 0xF800u) << 3) | ((b & 0x7F0u) << 2) |
           ((b & 0xEu) << 1) | (b & 1u);
}

static uint32_t ir_mcan_to_g4(uint32_t b)
{
    return ((b & 0x3FC00000u) >> 6) | ((b & 0x7C000u) >> 3) | ((b & 0x1FC0u) >> 2) |
           ((b & 0x1Cu) >> 1) | (b & 1u);
}

/* ILS: one G4 bit selects the line of a group of interrupt sources */
static const uint32_t g4_ils_groups[7] = {
    0x0000000F, 0x000000F0, 0x00000F00, 0x0000F000, 0x000F0000, 0x00700000, 0x3F800000,
};

static uint32_t ils_g4_to_mcan(uint32_t b)
{
    uint32_t v = 0;

    for (int i = 0; i < 7; i++) {
        if (b & BIT(i)) {
            v |= g4_ils_groups[i];
        }
    }
    return v;
}

static uint32_t ils_mcan_to_g4(uint32_t m)
{
    uint32_t v = 0;

    for (int i = 0; i < 7; i++) {
        if (m & g4_ils_groups[i]) {
            v |= BIT(i);
        }
    }
    return v;
}

static uint64_t fdcan_read_m(Stm32h7FdcanState *s, hwaddr off);
static void fdcan_write_m(Stm32h7FdcanState *s, hwaddr off, uint64_t val);

static uint64_t fdcan_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7FdcanState *s = opaque;
    int reg;

    if (!s->g4) {
        return fdcan_read_m(s, off);
    }
    if (off == G4_CKDIV) {
        return s->ckdiv;
    }
    reg = g4_reg(off);
    if (reg < 0) {
        return 0;
    }
    switch (reg) {
    case IR:
    case IE:
        return ir_mcan_to_g4(fdcan_read_m(s, reg));
    case ILS:
        return ils_mcan_to_g4(fdcan_read_m(s, reg));
    case GFC:
        return fdcan_read_m(s, reg);
    case TXBC:
        return R(s, TXBC) & BIT(24);
    }
    return fdcan_read_m(s, reg);
}

static void fdcan_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32h7FdcanState *s = opaque;
    int reg;

    if (!s->g4) {
        fdcan_write_m(s, off, val);
        return;
    }
    if (off == G4_CKDIV) {
        s->ckdiv = val;
        return;
    }
    reg = g4_reg(off);
    if (reg < 0) {
        return;
    }
    switch (reg) {
    case IR:
    case IE:
        val = ir_g4_to_mcan(val);
        break;
    case ILS:
        val = ils_g4_to_mcan(val);
        break;
    case TXBC:
        val &= BIT(24);
        break;
    }
    fdcan_write_m(s, reg, val);
}

/* ---- registers ---- */

static uint64_t fdcan_read_m(Stm32h7FdcanState *s, hwaddr off)
{
    switch (off) {
    case CREL: return 0x32141218;       /* M_CAN 3.2.1 */
    case ENDN: return 0x87654321;
    case PSR:  return 0x00000707;       /* LEC/DLEC: no change, error active */
    case TXFQS: return txfqs(s);
    }
    return off < sizeof(s->regs) ? s->regs[off / 4] : 0;
}

static void fdcan_write_m(Stm32h7FdcanState *s, hwaddr off, uint64_t val)
{
    uint32_t v;

    switch (off) {
    case IR:                            /* write 1 to clear */
        R(s, IR) &= ~val;
        update_irq(s);
        return;
    case TXBAR:
        R(s, TXBRP) |= val;
        R(s, TXBTO) &= ~val;
        transmit(s, val);
        return;
    case TXBCR:                         /* cancel: nothing pending survives transmit() */
        R(s, TXBCF) |= val & ~R(s, TXBRP);
        return;
    case RXF0A:
    case RXF1A: {
        hwaddr st = off == RXF0A ? RXF0S : RXF1S;
        uint32_t fifo_size = rx_size(s, off == RXF1A);
        uint32_t status = R(s, st);
        uint32_t fill = status & 0x7F;
        uint32_t idx = val & 0x3F;
        uint32_t get = (status >> 8) & 0x3F;
        uint32_t put = (status >> 16) & 0x3F;
        uint32_t released;

        if (fifo_size == 0) {
            return;
        }
        released = (idx + fifo_size - get) % fifo_size + 1;
        released = MIN(released, fill);
        fill -= released;
        get = (idx + 1) % fifo_size;
        R(s, st) = (status & ~0x013F3F7F) | fill | (get << 8) | (put << 16);
        R(s, off) = val;
        return;
    }
    case TXEFA: {
        uint32_t efs = R(s, TXEFS);
        uint32_t ef_size = txef_size(s);
        uint32_t fill = efs & 0x3F;
        uint32_t get = (efs >> 8) & 0x1F, put = (efs >> 16) & 0x1F;
        uint32_t idx = val & 0x1F, released;

        if (ef_size == 0) {
            return;
        }
        released = MIN((idx + ef_size - get) % ef_size + 1, fill);
        fill -= released;
        get = (idx + 1) % ef_size;
        R(s, TXEFS) = (efs & ~0x021F1F3F) | fill | (get << 8) | (put << 16);
        R(s, TXEFA) = val;
        return;
    }
    case CCCR:
        v = val;
        if (!(v & CCCR_INIT)) {
            v &= ~BIT(1);               /* CCE clears with INIT */
        }
        R(s, CCCR) = v;
        return;
    case RXF0S: case RXF1S: case TXEFS: case TXBRP: case TXBTO: case TXBCF:
    case CREL: case ENDN: case PSR: case TXFQS:
        return;                         /* read-only */
    }
    if (off < sizeof(s->regs)) {
        s->regs[off / 4] = val;
        if (off == IE || off == ILS || off == ILE) {
            update_irq(s);
        }
    }
}

static const MemoryRegionOps fdcan_ops = {
    .read = fdcan_read,
    .write = fdcan_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void fdcan_reset(DeviceState *dev)
{
    Stm32h7FdcanState *s = STM32H7_FDCAN(dev);

    memset(s->regs, 0, sizeof(s->regs));
    R(s, CCCR) = CCCR_INIT;
    R(s, 0x1C) = 0x00000A33;            /* NBTP */
    R(s, 0x0C) = 0x00000A33;            /* DBTP */
    R(s, XIDAM) = 0x1FFFFFFF;
    s->line_len = 0;
    s->rxq_head = s->rxq_count = 0;
    s->rx_next_ns = 0;
    if (s->rx_timer) {
        timer_del(s->rx_timer);
    }
}

static void fdcan_realize(DeviceState *dev, Error **errp)
{
    Stm32h7FdcanState *s = STM32H7_FDCAN(dev);

    if (!s->msg_ram) {
        error_setg(errp, "stm32h7-fdcan: msg-ram link not set");
        return;
    }
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rx_deliver, s);
    qemu_chr_fe_set_handlers(&s->chr, chr_can_receive, chr_receive, NULL, NULL, s, NULL, true);
}

static void fdcan_init(Object *obj)
{
    Stm32h7FdcanState *s = STM32H7_FDCAN(obj);

    memory_region_init_io(&s->mmio, obj, &fdcan_ops, s, TYPE_STM32H7_FDCAN, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[0]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[1]);
}

static const Property fdcan_props[] = {
    DEFINE_PROP_CHR("chardev", Stm32h7FdcanState, chr),
    DEFINE_PROP_LINK("msg-ram", Stm32h7FdcanState, msg_ram, TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_BOOL("g4", Stm32h7FdcanState, g4, false),
    DEFINE_PROP_UINT32("msg-ram-offset", Stm32h7FdcanState, ram_offset, 0),
    DEFINE_PROP_UINT32("rx-frame-ns", Stm32h7FdcanState, rx_frame_ns, 0),
};

static void fdcan_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = fdcan_realize;
    device_class_set_legacy_reset(dc, fdcan_reset);
    device_class_set_props(dc, fdcan_props);
}

static const TypeInfo fdcan_info = {
    .name = TYPE_STM32H7_FDCAN,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7FdcanState),
    .instance_init = fdcan_init,
    .class_init = fdcan_class_init,
};

static void fdcan_register(void)
{
    type_register_static(&fdcan_info);
}
type_init(fdcan_register)
