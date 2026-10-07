/*
 * STM32H7 QUADSPI controller (RM0399 chapter 23) with an attached serial NOR
 * flash.
 *
 * The controller runs indirect read/write, automatic status polling and the
 * memory-mapped read window. The flash behaves like a 32 MiB (default)
 * Winbond W25Q256JV-class part: JEDEC ID, an SFDP table (JESD216B basic
 * flash parameters, so SFDP-probing drivers configure themselves), status
 * registers 1-3 with WEL and the QE bit (SR2 bit 1), 3- and 4-byte addressing
 * (B7h/E9h and the 4-byte opcodes), 1-1-1/1-1-2/1-2-2/1-1-4/1-4-4 reads, page
 * program, 4K/32K/64K/chip erase and software reset. Every operation completes
 * at once (WIP never reads 1). Bus widths, dummy cycles and DDR are accepted
 * but have no effect.
 *
 * Contents start erased (0xFF), or come from the "drive" property (a raw
 * image of flash-size bytes), which programs and erases are written back to.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/ssi/stm32h7_qspi.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"

#define R_CR    0x00
#define R_DCR   0x04
#define R_SR    0x08
#define R_FCR   0x0C
#define R_DLR   0x10
#define R_CCR   0x14
#define R_AR    0x18
#define R_ABR   0x1C
#define R_DR    0x20
#define R_PSMKR 0x24
#define R_PSMAR 0x28
#define R_PIR   0x2C
#define R_LPTR  0x30

#define CR_EN      BIT(0)
#define CR_ABORT   BIT(1)
#define CR_TEIE    BIT(16)
#define CR_TCIE    BIT(17)
#define CR_FTIE    BIT(18)
#define CR_SMIE    BIT(19)
#define CR_TOIE    BIT(20)
#define CR_APMS    BIT(22)
#define CR_PMM     BIT(23)

#define SR_TEF  BIT(0)
#define SR_TCF  BIT(1)
#define SR_FTF  BIT(2)
#define SR_SMF  BIT(3)
#define SR_TOF  BIT(4)
#define SR_BUSY BIT(5)

#define CCR_INSTR(c)  ((c) & 0xFF)
#define CCR_ADMODE(c) (((c) >> 10) & 3)
#define CCR_DMODE(c)  (((c) >> 24) & 3)
#define CCR_FMODE(c)  (((c) >> 26) & 3)

enum { FMODE_WRITE, FMODE_READ, FMODE_POLL, FMODE_MMAP };

#define SR1_WEL BIT(1)
#define SR2_QE  BIT(1)
#define SR3_ADS BIT(0)

static const uint8_t jedec_id[3] = { 0xEF, 0x40, 0x19 };

/* SFDP: header, one parameter header (basic flash parameters, 16 DWORDs at
 * 0x80), and the table. */
static const uint32_t bfp[16] = {
    0xFFF320E5, /* 4K erase 20h; 3- or 4-byte addr; 1-1-2, 1-2-2, 1-1-4, 1-4-4 */
    0x0FFFFFFF, /* density: 256 Mbit (patched from flash-size) */
    0x6B08EB44, /* 1-4-4 EBh 4 wait + 2 mode clocks; 1-1-4 6Bh 8 wait */
    0xBB423B08, /* 1-1-2 3Bh 8 wait; 1-2-2 BBh 0 wait + 2 mode clocks */
    0xFFFFFFEE, /* no 2-2-2 / 4-4-4 */
    0xFFFF0000,
    0xFFFF0000,
    0x520F200C, /* erase types: 4K 20h, 32K 52h */
    0x0000D810, /* erase type 3: 64K D8h */
    0x00000000,
    0x00000080, /* page size 2^8 = 256 */
    0xFFFFFFFF,
    0xFFFFFFFF,
    0xFFFFFFFF,
    0x00400000, /* QER 100b: QE is SR2 bit 1, written with 31h */
    0x01000010, /* enter 4-byte: B7h; soft reset: 66h + 99h */
};

static uint8_t sfdp_byte(Stm32h7QspiState *s, uint32_t a)
{
    static const uint8_t head[16] = {
        'S', 'F', 'D', 'P', 0x06, 0x01, 0x00, 0xFF,
        0x00, 0x06, 0x01, 16, 0x80, 0x00, 0x00, 0xFF,
    };
    if (a < sizeof(head)) {
        return head[a];
    }
    if (a >= 0x80 && a < 0x80 + sizeof(bfp)) {
        uint32_t i = (a - 0x80) / 4;
        uint32_t v = bfp[i];
        if (i == 1) {
            v = (uint32_t)s->flash_size * 8 - 1;
        }
        return v >> (8 * ((a - 0x80) % 4));
    }
    return 0xFF;
}

static bool is_read(uint8_t c)
{
    switch (c) {
    case 0x03: case 0x0B: case 0x3B: case 0x6B: case 0xBB: case 0xEB:
    case 0x13: case 0x0C: case 0x3C: case 0x6C: case 0xBC: case 0xEC:
        return true;
    }
    return false;
}

static bool is_program(uint8_t c)
{
    return c == 0x02 || c == 0x32 || c == 0x38 || c == 0x12 || c == 0x34 ||
           c == 0x3E;
}

static void flash_writeback(Stm32h7QspiState *s, uint32_t off, uint32_t len)
{
    if (s->blk && len) {
        if (blk_pwrite(s->blk, off, len, s->storage + off, 0) < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-qspi: write back failed\n");
        }
    }
}

static uint8_t sr1(Stm32h7QspiState *s)
{
    return (s->sr1 & ~SR1_WEL) | (s->wel ? SR1_WEL : 0);
}

/* Byte idx of the data phase of a read-type command */
static uint8_t data_out(Stm32h7QspiState *s, uint32_t idx)
{
    uint8_t c = s->cmd;

    if (is_read(c)) {
        return s->storage[(s->addr + idx) & (s->flash_size - 1)];
    }
    switch (c) {
    case 0x9F: return jedec_id[idx % 3];
    case 0x5A: return sfdp_byte(s, (s->addr & 0xFFFFFF) + idx);
    case 0x05: return sr1(s);
    case 0x35: return s->sr2;
    case 0x15: return (s->sr3 & ~SR3_ADS) | (s->addr4 ? SR3_ADS : 0);
    }
    qemu_log_mask(LOG_UNIMP, "stm32h7-qspi: read command 0x%02x\n", c);
    return 0xFF;
}

static void erase(Stm32h7QspiState *s, uint32_t size)
{
    uint32_t a = s->addr & (s->flash_size - 1) & ~(size - 1);

    if (!s->wel) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-qspi: erase without WREN\n");
        return;
    }
    memset(s->storage + a, 0xFF, size);
    flash_writeback(s, a, size);
    s->wel = false;
}

/* Commands with no data phase, run when the command starts */
static void exec_nodata(Stm32h7QspiState *s)
{
    switch (s->cmd) {
    case 0x06: s->wel = true; break;
    case 0x04: s->wel = false; break;
    case 0xB7: s->addr4 = true; break;
    case 0xE9: s->addr4 = false; break;
    case 0x20: case 0x21: erase(s, 4 * KiB); break;
    case 0x52: case 0x5C: erase(s, 32 * KiB); break;
    case 0xD8: case 0xDC: erase(s, 64 * KiB); break;
    case 0xC7: case 0x60: s->addr = 0; erase(s, s->flash_size); break;
    case 0x66: break;
    case 0x99: s->wel = false; s->addr4 = false; break;
    case 0x50: break;   /* volatile SR write enable */
    default:
        qemu_log_mask(LOG_UNIMP, "stm32h7-qspi: command 0x%02x\n", s->cmd);
    }
}

/* Data written to the flash; status-register writes apply at the end */
static void data_in(Stm32h7QspiState *s, uint8_t b)
{
    uint32_t idx = s->done++;

    if (is_program(s->cmd)) {
        if (s->wel) {
            uint32_t a = (s->addr + idx) & (s->flash_size - 1);
            s->storage[a] &= b;
        }
    } else if (idx < sizeof(s->buf)) {
        s->buf[idx] = b;
    }
    if (s->done < s->len) {
        return;
    }
    /* last byte */
    switch (s->cmd) {
    case 0x01:
        s->sr1 = s->buf[0] & ~SR1_WEL;
        if (s->len > 1) {
            s->sr2 = s->buf[1];
        }
        break;
    case 0x31: s->sr2 = s->buf[0]; break;
    case 0x11: s->sr3 = s->buf[0]; break;
    default:
        if (is_program(s->cmd)) {
            if (!s->wel) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "stm32h7-qspi: program without WREN\n");
            } else {
                uint32_t a = s->addr & (s->flash_size - 1);
                flash_writeback(s, a, MIN(s->len, s->flash_size - a));
            }
        } else {
            qemu_log_mask(LOG_UNIMP, "stm32h7-qspi: write command 0x%02x\n",
                          s->cmd);
        }
    }
    s->wel = false;
    s->active = false;
    s->sr |= SR_TCF;
}

static uint32_t thresh(Stm32h7QspiState *s)
{
    return ((s->cr >> 8) & 0x1F) + 1;
}

static void fifo_fill(Stm32h7QspiState *s)
{
    while (s->active && s->fifo_level < STM32H7_QSPI_FIFO && s->done < s->len) {
        s->fifo[s->fifo_level++] = data_out(s, s->done++);
    }
    if (s->active && s->done >= s->len) {
        s->active = false;
        s->sr |= SR_TCF;
    }
}

static uint32_t sr_value(Stm32h7QspiState *s)
{
    uint32_t v = s->sr & (SR_TEF | SR_TCF | SR_SMF | SR_TOF);
    bool reading = CCR_FMODE(s->ccr) == FMODE_READ;

    if (reading) {
        if (s->fifo_level >= thresh(s) || (!s->active && s->fifo_level)) {
            v |= SR_FTF;
        }
    } else if (s->active && CCR_FMODE(s->ccr) == FMODE_WRITE && s->len) {
        v |= SR_FTF;
    }
    if (s->active || s->fifo_level) {
        v |= SR_BUSY;
    }
    return v | (MIN(s->fifo_level, 63) << 8);
}

static void update_irq(Stm32h7QspiState *s)
{
    uint32_t v = sr_value(s);
    bool lvl = ((s->cr & CR_TEIE) && (v & SR_TEF)) ||
               ((s->cr & CR_TCIE) && (v & SR_TCF)) ||
               ((s->cr & CR_FTIE) && (v & SR_FTF)) ||
               ((s->cr & CR_SMIE) && (v & SR_SMF)) ||
               ((s->cr & CR_TOIE) && (v & SR_TOF));
    qemu_set_irq(s->irq, lvl);
}

static void stop(Stm32h7QspiState *s)
{
    s->active = false;
    s->fifo_level = 0;
    s->len = s->done = 0;
}

static void poll_status(Stm32h7QspiState *s)
{
    uint32_t n = MIN(s->dlr + 1, 4), v = 0;

    for (uint32_t i = 0; i < n; i++) {
        v |= (uint32_t)data_out(s, i) << (8 * i);
    }
    s->fifo_level = 0;
    for (uint32_t i = 0; i < n; i++) {
        s->fifo[s->fifo_level++] = v >> (8 * i);
    }
    if ((v & s->psmkr) == (s->psmar & s->psmkr)) {
        s->sr |= SR_SMF;
        if (s->cr & CR_APMS) {
            s->active = false;
            s->sr |= SR_TCF;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-qspi: status poll 0x%x never "
                      "matches 0x%x/0x%x\n", v, s->psmar, s->psmkr);
    }
}

/* A command starts on a CCR write (no address phase) or an AR write */
static void start(Stm32h7QspiState *s)
{
    uint32_t fm = CCR_FMODE(s->ccr);

    stop(s);
    if (!(s->cr & CR_EN) || fm == FMODE_MMAP) {
        return;
    }
    s->cmd = CCR_INSTR(s->ccr);
    s->addr = s->ar;
    s->active = true;
    if (CCR_DMODE(s->ccr) == 0) {
        exec_nodata(s);
        s->active = false;
        s->sr |= SR_TCF;
        return;
    }
    s->len = s->dlr + 1;
    switch (fm) {
    case FMODE_READ:
        fifo_fill(s);
        break;
    case FMODE_POLL:
        poll_status(s);
        break;
    default:
        break;      /* indirect write: data arrives through DR */
    }
}

static uint64_t qspi_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7QspiState *s = opaque;
    uint64_t v = 0;

    switch (off) {
    case R_CR:    v = s->cr; break;
    case R_DCR:   v = s->dcr; break;
    case R_SR:    v = sr_value(s); break;
    case R_DLR:   v = s->dlr; break;
    case R_CCR:   v = s->ccr; break;
    case R_AR:    v = s->ar; break;
    case R_ABR:   v = s->abr; break;
    case R_PSMKR: v = s->psmkr; break;
    case R_PSMAR: v = s->psmar; break;
    case R_PIR:   v = s->pir; break;
    case R_LPTR:  v = s->lptr; break;
    case R_DR:
        for (unsigned i = 0; i < size; i++) {
            if (s->fifo_level) {
                v |= (uint64_t)s->fifo[0] << (8 * i);
                memmove(s->fifo, s->fifo + 1, --s->fifo_level);
            }
        }
        if (CCR_FMODE(s->ccr) == FMODE_READ) {
            fifo_fill(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-qspi: read 0x%" HWADDR_PRIx "\n",
                      off);
    }
    update_irq(s);
    return v;
}

static void qspi_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7QspiState *s = opaque;

    switch (off) {
    case R_CR:
        if (v & CR_ABORT) {
            stop(s);
            s->sr |= SR_TCF;
        }
        s->cr = v & ~CR_ABORT;
        if (!(s->cr & CR_EN)) {
            stop(s);
        }
        break;
    case R_DCR:   s->dcr = v; break;
    case R_FCR:   s->sr &= ~(v & (SR_TEF | SR_TCF | SR_SMF | SR_TOF)); break;
    case R_DLR:   s->dlr = v; break;
    case R_CCR:
        s->ccr = v;
        if (CCR_ADMODE(v) == 0) {
            start(s);
        } else if (s->active && CCR_FMODE(v) != FMODE_WRITE) {
            stop(s);    /* a read restarts on the AR write that follows */
        }
        /* a pending indirect write keeps going: HAL rewrites CCR (FMODE)
         * between the command and the data without rewriting AR */
        break;
    case R_AR:
        s->ar = v;
        if (CCR_ADMODE(s->ccr) != 0) {
            start(s);
        }
        break;
    case R_ABR:   s->abr = v; break;
    case R_PSMKR: s->psmkr = v; break;
    case R_PSMAR: s->psmar = v; break;
    case R_PIR:   s->pir = v; break;
    case R_LPTR:  s->lptr = v; break;
    case R_DR:
        if (!s->active && CCR_FMODE(s->ccr) == FMODE_WRITE && CCR_DMODE(s->ccr)) {
            start(s);   /* in indirect write mode, data starts the command */
        }
        for (unsigned i = 0; i < size && s->active &&
             CCR_FMODE(s->ccr) == FMODE_WRITE && s->done < s->len; i++) {
            data_in(s, v >> (8 * i));
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-qspi: write 0x%" HWADDR_PRIx "\n",
                      off);
    }
    update_irq(s);
}

static const MemoryRegionOps qspi_ops = {
    .read = qspi_read,
    .write = qspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t mapped_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7QspiState *s = opaque;
    uint64_t v = 0;

    for (unsigned i = 0; i < size; i++) {
        v |= (uint64_t)s->storage[(off + i) & (s->flash_size - 1)] << (8 * i);
    }
    return v;
}

static void mapped_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-qspi: write to the mapped window\n");
}

static const MemoryRegionOps mapped_ops = {
    .read = mapped_read,
    .write = mapped_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void qspi_reset(DeviceState *dev)
{
    Stm32h7QspiState *s = STM32H7_QSPI(dev);

    s->cr = s->dcr = s->sr = s->dlr = s->ccr = s->ar = s->abr = 0;
    s->psmkr = s->psmar = s->pir = s->lptr = 0;
    s->sr1 = s->sr2 = s->sr3 = 0;
    s->wel = s->addr4 = false;
    stop(s);
}

static void qspi_realize(DeviceState *dev, Error **errp)
{
    Stm32h7QspiState *s = STM32H7_QSPI(dev);

    if (!is_power_of_2(s->flash_size)) {
        error_setg(errp, "stm32h7-qspi: flash-size must be a power of 2");
        return;
    }
    s->storage = g_malloc(s->flash_size);
    memset(s->storage, 0xFF, s->flash_size);
    if (s->blk) {
        int64_t len = blk_getlength(s->blk);
        uint64_t perm = BLK_PERM_CONSISTENT_READ |
                        (blk_supports_write_perm(s->blk) ? BLK_PERM_WRITE : 0);

        if (blk_set_perm(s->blk, perm, BLK_PERM_ALL, errp) < 0) {
            return;
        }
        if (len > 0 && blk_pread(s->blk, 0, MIN(len, s->flash_size),
                                 s->storage, 0) < 0) {
            error_setg(errp, "stm32h7-qspi: cannot read the drive");
            return;
        }
    }
    memory_region_init_io(&s->regs, OBJECT(s), &qspi_ops, s, "stm32h7-qspi", 0x400);
    memory_region_init_io(&s->mapped, OBJECT(s), &mapped_ops, s,
                          "stm32h7-qspi.mapped", s->flash_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->regs);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mapped);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static const Property qspi_props[] = {
    DEFINE_PROP_UINT32("flash-size", Stm32h7QspiState, flash_size, 32 * MiB),
    DEFINE_PROP_DRIVE("drive", Stm32h7QspiState, blk),
};

static void qspi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = qspi_realize;
    device_class_set_legacy_reset(dc, qspi_reset);
    device_class_set_props(dc, qspi_props);
}

static const TypeInfo qspi_info = {
    .name = TYPE_STM32H7_QSPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7QspiState),
    .class_init = qspi_class_init,
};

static void qspi_register(void)
{
    type_register_static(&qspi_info);
}
type_init(qspi_register)
