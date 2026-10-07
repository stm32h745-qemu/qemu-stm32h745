/*
 * STM32H7 DMA controller (DMA1/DMA2: 8 streams, RM0399 chapter 15) and
 * DMAMUX1 (16 request channels, chapter 17).
 *
 * stm32h7-dma: each stream has a request input (GPIO in "req", one per
 * stream; DMAMUX1 drives them) and an interrupt. While a stream is enabled
 * and its request is high, it moves one data item per request (peripheral
 * to memory or memory to peripheral) on the system bus, served from a bottom
 * half right after the request rises; memory to memory runs the whole block
 * when enabled. NDTR counts down; half and full
 * transfer flags, circular mode and double-buffer mode (DBM/CT) work as on
 * hardware; a bus error sets TEIF and disables the stream.
 * Not modeled: the FIFO (FTH/DMDIS are accepted; data sizes must match, items
 * move at PSIZE), bursts, PINCOS, priorities (requests are served in order),
 * flow control by the peripheral.
 *
 * stm32h7-dmamux: 128 request inputs (GPIO in "req", indexed by request ID,
 * 1-115 on this device) routed by CxCR.DMAREQ_ID to 16 outputs (GPIO out
 * "out": 0-7 DMA1 streams 0-7, 8-15 DMA2 streams 0-7). Synchronization and
 * the request generators are not modeled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/main-loop.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "qom/object.h"

#define TYPE_STM32H7_DMA "stm32h7-dma"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7DmaState, STM32H7_DMA)
#define TYPE_STM32H7_DMAMUX "stm32h7-dmamux"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7DmamuxState, STM32H7_DMAMUX)

#define NSTREAMS 8

typedef struct Stream {
    uint32_t cr, ndtr, par, m0ar, m1ar, fcr;
    uint32_t ndtr0;         /* NDTR at enable, for reload */
    uint32_t idx;           /* items done in this pass */
    bool req;
} Stream;

struct Stm32h7DmaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[NSTREAMS];
    Stream st[NSTREAMS];
    uint32_t isr[2];        /* LISR, HISR */
    bool busy;
    QEMUBH *bh;             /* serves requests outside the requester's MMIO */
};

#define CR_EN     BIT(0)
#define CR_DMEIE  BIT(1)
#define CR_TEIE   BIT(2)
#define CR_HTIE   BIT(3)
#define CR_TCIE   BIT(4)
#define CR_DIR(c) (((c) >> 6) & 3)
#define CR_CIRC   BIT(8)
#define CR_PINC   BIT(9)
#define CR_MINC   BIT(10)
#define CR_PSIZE(c) (((c) >> 11) & 3)
#define CR_MSIZE(c) (((c) >> 13) & 3)
#define CR_DBM    BIT(18)
#define CR_CT     BIT(19)
#define FCR_FEIE  BIT(7)

#define F_FEIF  BIT(0)
#define F_DMEIF BIT(2)
#define F_TEIF  BIT(3)
#define F_HTIF  BIT(4)
#define F_TCIF  BIT(5)

static const unsigned flag_shift[4] = { 0, 6, 16, 22 };

static uint32_t flags(Stm32h7DmaState *s, int n)
{
    return (s->isr[n / 4] >> flag_shift[n % 4]) & 0x3D;
}

static void set_flag(Stm32h7DmaState *s, int n, uint32_t f)
{
    s->isr[n / 4] |= f << flag_shift[n % 4];
}

static void update_irq(Stm32h7DmaState *s, int n)
{
    Stream *t = &s->st[n];
    uint32_t f = flags(s, n);
    bool lvl = ((f & F_TCIF) && (t->cr & CR_TCIE)) ||
               ((f & F_HTIF) && (t->cr & CR_HTIE)) ||
               ((f & F_TEIF) && (t->cr & CR_TEIE)) ||
               ((f & F_DMEIF) && (t->cr & CR_DMEIE)) ||
               ((f & F_FEIF) && (t->fcr & FCR_FEIE));

    qemu_set_irq(s->irq[n], lvl);
}

/* Move one item; false on a bus error (stream disabled) */
static bool move_one(Stm32h7DmaState *s, int n)
{
    Stream *t = &s->st[n];
    unsigned size = 1 << CR_PSIZE(t->cr);
    uint32_t mbase = (t->cr & CR_CT) ? t->m1ar : t->m0ar;
    hwaddr pa = t->par + ((t->cr & CR_PINC) ? t->idx * size : 0);
    hwaddr ma = mbase + ((t->cr & CR_MINC) ? t->idx * size : 0);
    hwaddr src, dst;
    uint8_t buf[4];
    MemTxResult r;

    switch (CR_DIR(t->cr)) {
    case 0: src = pa; dst = ma; break;      /* peripheral to memory */
    case 1: src = ma; dst = pa; break;      /* memory to peripheral */
    case 2: src = pa; dst = ma; break;      /* memory to memory: PAR is source */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-dma: stream %d DIR=3\n", n);
        return false;
    }
    r = address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                           buf, size);
    if (r == MEMTX_OK) {
        r = address_space_write(&address_space_memory, dst, MEMTXATTRS_UNSPECIFIED,
                                buf, size);
    }
    if (r != MEMTX_OK) {
        set_flag(s, n, F_TEIF);
        t->cr &= ~CR_EN;
        return false;
    }
    t->idx++;
    t->ndtr--;
    if (t->ndtr == t->ndtr0 / 2) {
        set_flag(s, n, F_HTIF);
    }
    if (t->ndtr == 0) {
        set_flag(s, n, F_TCIF);
        if (t->cr & (CR_CIRC | CR_DBM)) {
            t->ndtr = t->ndtr0;
            t->idx = 0;
            if (t->cr & CR_DBM) {
                t->cr ^= CR_CT;
            }
        } else {
            t->cr &= ~CR_EN;
        }
    }
    return true;
}

/* Serve requests until none is pending; re-entry (a transfer raising a
 * request again) is folded into the running loop */
static void service(Stm32h7DmaState *s)
{
    bool progress = true;

    if (s->busy) {
        return;
    }
    s->busy = true;
    while (progress) {
        progress = false;
        for (int n = 0; n < NSTREAMS; n++) {
            Stream *t = &s->st[n];

            if ((t->cr & CR_EN) && t->ndtr && (t->req || CR_DIR(t->cr) == 2)) {
                if (move_one(s, n)) {
                    progress = true;
                }
                update_irq(s, n);
            }
        }
    }
    s->busy = false;
}

static void service_bh(void *opaque)
{
    service(opaque);
}

/* A request is served from a bottom half: the requesting peripheral is
 * usually inside its own register access when it raises the request */
static void req_in(void *opaque, int n, int level)
{
    Stm32h7DmaState *s = opaque;

    s->st[n].req = level;
    if (level && !s->busy) {
        qemu_bh_schedule(s->bh);
    }
}

static uint64_t dma_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7DmaState *s = opaque;

    if (off < 0x10) {
        return off < 0x08 ? s->isr[off / 4] : 0;
    }
    off -= 0x10;
    if (off / 0x18 >= NSTREAMS) {
        return 0;
    }
    Stream *t = &s->st[off / 0x18];
    switch (off % 0x18) {
    case 0x00: return t->cr;
    case 0x04: return t->ndtr;
    case 0x08: return t->par;
    case 0x0C: return t->m0ar;
    case 0x10: return t->m1ar;
    case 0x14: return t->fcr | (0x4 << 3);  /* FS = 100: FIFO empty */
    }
    return 0;
}

static void dma_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7DmaState *s = opaque;

    if (off < 0x10) {
        if (off == 0x08 || off == 0x0C) {        /* LIFCR, HIFCR */
            s->isr[(off - 0x08) / 4] &= ~v;
            for (int n = 0; n < NSTREAMS; n++) {
                update_irq(s, n);
            }
        }
        return;
    }
    off -= 0x10;
    if (off / 0x18 >= NSTREAMS) {
        return;
    }
    int n = off / 0x18;
    Stream *t = &s->st[n];
    switch (off % 0x18) {
    case 0x00:
        if ((v & CR_EN) && !(t->cr & CR_EN)) {
            if (CR_PSIZE(v) != CR_MSIZE(v)) {
                qemu_log_mask(LOG_UNIMP, "stm32h7-dma: stream %d packing "
                              "(PSIZE != MSIZE) moves PSIZE items\n", n);
            }
            t->ndtr0 = t->ndtr;
            t->idx = 0;
        }
        t->cr = v;
        update_irq(s, n);
        qemu_bh_schedule(s->bh);
        return;
    case 0x04: if (!(t->cr & CR_EN)) { t->ndtr = v & 0xFFFF; } break;
    case 0x08: if (!(t->cr & CR_EN)) { t->par = v; } break;
    case 0x0C: t->m0ar = v; break;
    case 0x10: t->m1ar = v; break;
    case 0x14: t->fcr = v & 0x87; break;
    }
}

static const MemoryRegionOps dma_ops = {
    .read = dma_read,
    .write = dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void dma_reset(DeviceState *dev)
{
    Stm32h7DmaState *s = STM32H7_DMA(dev);

    for (int n = 0; n < NSTREAMS; n++) {
        bool req = s->st[n].req;

        memset(&s->st[n], 0, sizeof(Stream));
        s->st[n].req = req;
        s->st[n].fcr = 0x21;
        update_irq(s, n);
    }
    s->isr[0] = s->isr[1] = 0;
}

static void dma_init(Object *obj)
{
    Stm32h7DmaState *s = STM32H7_DMA(obj);

    memory_region_init_io(&s->iomem, obj, &dma_ops, s, "stm32h7-dma", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    for (int n = 0; n < NSTREAMS; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[n]);
    }
    qdev_init_gpio_in_named(DEVICE(s), req_in, "req", NSTREAMS);
    s->bh = qemu_bh_new_guarded(service_bh, s, &DEVICE(s)->mem_reentrancy_guard);
}

static void dma_class_init(ObjectClass *klass, void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), dma_reset);
}

/* DMAMUX1 */

#define MUX_CH  16
#define MUX_REQ 128

struct Stm32h7DmamuxState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq out[MUX_CH];
    uint32_t ccr[MUX_CH];
    uint32_t rgcr[8];
    bool in[MUX_REQ];
};

static void mux_route(Stm32h7DmamuxState *s, int c)
{
    unsigned id = s->ccr[c] & 0x7F;

    qemu_set_irq(s->out[c], id && s->in[id]);
}

static void mux_in(void *opaque, int id, int level)
{
    Stm32h7DmamuxState *s = opaque;

    s->in[id] = level;
    for (int c = 0; c < MUX_CH; c++) {
        if ((s->ccr[c] & 0x7F) == id) {
            mux_route(s, c);
        }
    }
}

static uint64_t mux_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7DmamuxState *s = opaque;

    if (off < 4 * MUX_CH) {
        return s->ccr[off / 4];
    }
    if (off >= 0x100 && off < 0x120) {
        return s->rgcr[(off - 0x100) / 4];
    }
    return 0;
}

static void mux_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7DmamuxState *s = opaque;

    if (off < 4 * MUX_CH) {
        s->ccr[off / 4] = v;
        mux_route(s, off / 4);
    } else if (off >= 0x100 && off < 0x120) {
        s->rgcr[(off - 0x100) / 4] = v;
    }
}

static const MemoryRegionOps mux_ops = {
    .read = mux_read,
    .write = mux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void mux_reset(DeviceState *dev)
{
    Stm32h7DmamuxState *s = STM32H7_DMAMUX(dev);

    memset(s->ccr, 0, sizeof(s->ccr));
    memset(s->rgcr, 0, sizeof(s->rgcr));
    for (int c = 0; c < MUX_CH; c++) {
        mux_route(s, c);
    }
}

static void mux_init(Object *obj)
{
    Stm32h7DmamuxState *s = STM32H7_DMAMUX(obj);

    memory_region_init_io(&s->iomem, obj, &mux_ops, s, "stm32h7-dmamux", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    qdev_init_gpio_in_named(DEVICE(s), mux_in, "req", MUX_REQ);
    qdev_init_gpio_out_named(DEVICE(s), s->out, "out", MUX_CH);
}

static void mux_class_init(ObjectClass *klass, void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), mux_reset);
}

static const TypeInfo types[] = {
    {
        .name = TYPE_STM32H7_DMA,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h7DmaState),
        .instance_init = dma_init,
        .class_init = dma_class_init,
    }, {
        .name = TYPE_STM32H7_DMAMUX,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h7DmamuxState),
        .instance_init = mux_init,
        .class_init = mux_class_init,
    },
};
DEFINE_TYPES(types)
