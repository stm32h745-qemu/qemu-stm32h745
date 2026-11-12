/*
 * STM32G4 DMA controller (RM0440 chapter 12) and DMAMUX (chapter 13).
 *
 * DMA: 8 channels, peripheral-to-memory, memory-to-peripheral and
 * memory-to-memory, 8/16/32 bit sizes, address increment, circular mode,
 * half-transfer/transfer-complete/transfer-error flags and interrupts. A
 * peripheral request (level, from the DMAMUX) moves one unit; the transfer
 * happens at once, so a peripheral that drops its request when its data
 * register is read (the ADC, USART) sees one unit per request.
 *
 * DMAMUX: routes the 16 channel inputs (DMAREQ_ID in CxCR) from the
 * peripheral request lines ("req" inputs numbered by request ID) to the DMA
 * channel request inputs. Synchronization and request generators are not
 * modeled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "exec/address-spaces.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_STM32G4_DMA "stm32g4-dma"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4DmaState, STM32G4_DMA)
#define TYPE_STM32G4_DMAMUX "stm32g4-dmamux"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4DmamuxState, STM32G4_DMAMUX)

#define NUM_CH     8
#define MUX_CH     16
#define MUX_REQS   128

#define CCR_EN      BIT(0)
#define CCR_TCIE    BIT(1)
#define CCR_HTIE    BIT(2)
#define CCR_TEIE    BIT(3)
#define CCR_DIR     BIT(4)
#define CCR_CIRC    BIT(5)
#define CCR_PINC    BIT(6)
#define CCR_MINC    BIT(7)
#define CCR_M2M     BIT(14)

#define F_GIF  1
#define F_TCIF 2
#define F_HTIF 4
#define F_TEIF 8

typedef struct DmaChannel {
    uint32_t ccr, cndtr, cpar, cmar;
    uint32_t total, cur_par, cur_mar;   /* latched at enable */
    bool req;
} DmaChannel;

struct Stm32g4DmaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[NUM_CH];
    uint32_t isr;
    DmaChannel ch[NUM_CH];
    bool busy;
};

static void update_irq(Stm32g4DmaState *s)
{
    for (int i = 0; i < NUM_CH; i++) {
        uint32_t f = (s->isr >> (4 * i)) & 0xF;
        uint32_t c = s->ch[i].ccr;

        qemu_set_irq(s->irq[i], ((f & F_TCIF) && (c & CCR_TCIE)) ||
                                ((f & F_HTIF) && (c & CCR_HTIE)) ||
                                ((f & F_TEIF) && (c & CCR_TEIE)));
    }
}

static void set_flags(Stm32g4DmaState *s, int i, uint32_t f)
{
    s->isr |= (f | F_GIF) << (4 * i);
}

static unsigned size_of(uint32_t code)
{
    return 1u << (code & 3);        /* 0: 8 bit, 1: 16 bit, 2: 32 bit */
}

static void transfer_unit(Stm32g4DmaState *s, int i)
{
    DmaChannel *c = &s->ch[i];
    unsigned psz = size_of(c->ccr >> 8), msz = size_of(c->ccr >> 10);
    uint8_t buf[8] = { 0 };
    uint64_t v = 0;
    MemTxResult r;

    if (c->ccr & CCR_M2M) {
        /* memory-to-memory: CPAR is the source (DIR = 0) or the destination */
        hwaddr src = (c->ccr & CCR_DIR) ? c->cur_mar : c->cur_par;
        hwaddr dst = (c->ccr & CCR_DIR) ? c->cur_par : c->cur_mar;
        unsigned ssz = (c->ccr & CCR_DIR) ? msz : psz;
        unsigned dsz = (c->ccr & CCR_DIR) ? psz : msz;

        r = address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED, buf, ssz);
        if (r == MEMTX_OK) {
            v = ldn_le_p(buf, ssz);
            memset(buf, 0, sizeof(buf));
            stn_le_p(buf, dsz, v);
            r = address_space_write(&address_space_memory, dst, MEMTXATTRS_UNSPECIFIED,
                                    buf, dsz);
        }
    } else if (c->ccr & CCR_DIR) {          /* memory to peripheral */
        r = address_space_read(&address_space_memory, c->cur_mar, MEMTXATTRS_UNSPECIFIED,
                               buf, msz);
        if (r == MEMTX_OK) {
            v = ldn_le_p(buf, msz);
            memset(buf, 0, sizeof(buf));
            stn_le_p(buf, psz, v);
            r = address_space_write(&address_space_memory, c->cur_par,
                                    MEMTXATTRS_UNSPECIFIED, buf, psz);
        }
    } else {                                /* peripheral to memory */
        r = address_space_read(&address_space_memory, c->cur_par, MEMTXATTRS_UNSPECIFIED,
                               buf, psz);
        if (r == MEMTX_OK) {
            v = ldn_le_p(buf, psz);
            memset(buf, 0, sizeof(buf));
            stn_le_p(buf, msz, v);
            r = address_space_write(&address_space_memory, c->cur_mar,
                                    MEMTXATTRS_UNSPECIFIED, buf, msz);
        }
    }
    if (r != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32g4-dma: channel %d bus error\n", i + 1);
        c->ccr &= ~CCR_EN;
        set_flags(s, i, F_TEIF);
        return;
    }
    if (c->ccr & CCR_PINC) {
        c->cur_par += psz;
    }
    if (c->ccr & CCR_MINC) {
        c->cur_mar += msz;
    }
    c->cndtr--;
    if (c->total > 1 && (c->total - c->cndtr) * 2 == c->total) {
        set_flags(s, i, F_HTIF);
    }
    if (c->cndtr == 0) {
        set_flags(s, i, F_TCIF);
        if (c->ccr & CCR_CIRC) {
            c->cndtr = c->total;
            c->cur_par = c->cpar;
            c->cur_mar = c->cmar;
        } else {
            c->ccr &= ~CCR_EN;
        }
    }
}

static void serve(Stm32g4DmaState *s, int i)
{
    DmaChannel *c = &s->ch[i];

    if (s->busy) {
        return;
    }
    s->busy = true;
    if (c->ccr & CCR_M2M) {
        while ((c->ccr & CCR_EN) && c->cndtr) {
            transfer_unit(s, i);
        }
    } else {
        while ((c->ccr & CCR_EN) && c->cndtr && c->req) {
            transfer_unit(s, i);
        }
    }
    s->busy = false;
    update_irq(s);
}

static void req_set(void *opaque, int n, int level)
{
    Stm32g4DmaState *s = opaque;

    s->ch[n].req = level;
    if (level) {
        serve(s, n);
    }
}

static uint64_t dma_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4DmaState *s = opaque;

    if (off == 0x00) {
        return s->isr;
    }
    if (off >= 0x08 && off < 0x08 + 0x14 * NUM_CH) {
        DmaChannel *c = &s->ch[(off - 8) / 0x14];

        switch ((off - 8) % 0x14) {
        case 0x00: return c->ccr;
        case 0x04: return c->cndtr;
        case 0x08: return c->cpar;
        case 0x0C: return c->cmar;
        }
    }
    return 0;
}

static void dma_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4DmaState *s = opaque;

    if (off == 0x04) {
        for (int i = 0; i < NUM_CH; i++) {
            uint32_t f = (val >> (4 * i)) & 0xF;

            if (f & F_GIF) {
                s->isr &= ~(0xFu << (4 * i));
            } else {
                s->isr &= ~(f << (4 * i));
            }
        }
        update_irq(s);
        return;
    }
    if (off >= 0x08 && off < 0x08 + 0x14 * NUM_CH) {
        int i = (off - 8) / 0x14;
        DmaChannel *c = &s->ch[i];
        bool en = c->ccr & CCR_EN;

        switch ((off - 8) % 0x14) {
        case 0x00:
            if (!(val & CCR_EN) && en) {
                c->ccr = val;
            } else if ((val & CCR_EN) && !en) {
                c->ccr = val;
                c->total = c->cndtr;
                c->cur_par = c->cpar;
                c->cur_mar = c->cmar;
                serve(s, i);
            } else if (en) {
                /* only the interrupt enables can change while enabled */
                c->ccr = (c->ccr & ~0xEu) | (val & 0xE);
            } else {
                c->ccr = val;
            }
            update_irq(s);
            return;
        case 0x04: if (!en) { c->cndtr = val & 0xFFFF; } return;
        case 0x08: if (!en) { c->cpar = val; } return;
        case 0x0C: if (!en) { c->cmar = val; } return;
        }
    }
}

static const MemoryRegionOps dma_ops = {
    .read = dma_read,
    .write = dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void dma_reset(DeviceState *dev)
{
    Stm32g4DmaState *s = STM32G4_DMA(dev);

    s->isr = 0;
    for (int i = 0; i < NUM_CH; i++) {
        bool req = s->ch[i].req;

        memset(&s->ch[i], 0, sizeof(s->ch[i]));
        s->ch[i].req = req;
    }
    update_irq(s);
}

static void dma_init(Object *obj)
{
    Stm32g4DmaState *s = STM32G4_DMA(obj);

    memory_region_init_io(&s->iomem, obj, &dma_ops, s, TYPE_STM32G4_DMA, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (int i = 0; i < NUM_CH; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_in_named(DEVICE(obj), req_set, "req", NUM_CH);
}

static const VMStateDescription vmstate_dma_ch = {
    .name = "stm32g4-dma-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ccr, DmaChannel),
        VMSTATE_UINT32(cndtr, DmaChannel),
        VMSTATE_UINT32(cpar, DmaChannel),
        VMSTATE_UINT32(cmar, DmaChannel),
        VMSTATE_UINT32(total, DmaChannel),
        VMSTATE_UINT32(cur_par, DmaChannel),
        VMSTATE_UINT32(cur_mar, DmaChannel),
        VMSTATE_BOOL(req, DmaChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_dma = {
    .name = TYPE_STM32G4_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(isr, Stm32g4DmaState),
        VMSTATE_STRUCT_ARRAY(ch, Stm32g4DmaState, NUM_CH, 1, vmstate_dma_ch, DmaChannel),
        VMSTATE_END_OF_LIST()
    }
};

static void dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, dma_reset);
    dc->vmsd = &vmstate_dma;
}

static const TypeInfo dma_info = {
    .name = TYPE_STM32G4_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4DmaState),
    .instance_init = dma_init,
    .class_init = dma_class_init,
};

/* ---- DMAMUX ---- */

struct Stm32g4DmamuxState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq out[MUX_CH];
    uint32_t cr[MUX_CH];
    uint32_t rg[4];
    bool in[MUX_REQS];
};

static void mux_update(Stm32g4DmamuxState *s, int ch)
{
    uint32_t id = s->cr[ch] & 0x7F;

    qemu_set_irq(s->out[ch], id && id < MUX_REQS && s->in[id]);
}

static void mux_req(void *opaque, int n, int level)
{
    Stm32g4DmamuxState *s = opaque;

    s->in[n] = level;
    for (int ch = 0; ch < MUX_CH; ch++) {
        if ((s->cr[ch] & 0x7F) == n) {
            mux_update(s, ch);
        }
    }
}

static uint64_t mux_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4DmamuxState *s = opaque;

    if (off < 4 * MUX_CH) {
        return s->cr[off / 4];
    }
    if (off >= 0x100 && off < 0x110) {
        return s->rg[(off - 0x100) / 4];
    }
    return 0;      /* status registers: no synchronization overrun */
}

static void mux_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4DmamuxState *s = opaque;

    if (off < 4 * MUX_CH) {
        s->cr[off / 4] = val;
        if (val & (BIT(16) | BIT(9))) {
            qemu_log_mask(LOG_UNIMP, "stm32g4-dmamux: synchronization/event generation "
                                     "not modeled\n");
        }
        mux_update(s, off / 4);
    } else if (off >= 0x100 && off < 0x110) {
        s->rg[(off - 0x100) / 4] = val;
        qemu_log_mask(LOG_UNIMP, "stm32g4-dmamux: request generators not modeled\n");
    }
}

static const MemoryRegionOps mux_ops = {
    .read = mux_read,
    .write = mux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mux_reset(DeviceState *dev)
{
    Stm32g4DmamuxState *s = STM32G4_DMAMUX(dev);

    memset(s->cr, 0, sizeof(s->cr));
    memset(s->rg, 0, sizeof(s->rg));
    for (int ch = 0; ch < MUX_CH; ch++) {
        mux_update(s, ch);
    }
}

static void mux_init(Object *obj)
{
    Stm32g4DmamuxState *s = STM32G4_DMAMUX(obj);

    memory_region_init_io(&s->iomem, obj, &mux_ops, s, TYPE_STM32G4_DMAMUX, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(DEVICE(obj), mux_req, "req", MUX_REQS);
    qdev_init_gpio_out_named(DEVICE(obj), s->out, "out", MUX_CH);
}

static const VMStateDescription vmstate_mux = {
    .name = TYPE_STM32G4_DMAMUX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cr, Stm32g4DmamuxState, MUX_CH),
        VMSTATE_UINT32_ARRAY(rg, Stm32g4DmamuxState, 4),
        VMSTATE_BOOL_ARRAY(in, Stm32g4DmamuxState, MUX_REQS),
        VMSTATE_END_OF_LIST()
    }
};

static void mux_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mux_reset);
    dc->vmsd = &vmstate_mux;
}

static const TypeInfo mux_info = {
    .name = TYPE_STM32G4_DMAMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4DmamuxState),
    .instance_init = mux_init,
    .class_init = mux_class_init,
};

static void stm32g4_dma_register(void)
{
    type_register_static(&dma_info);
    type_register_static(&mux_info);
}
type_init(stm32g4_dma_register)
