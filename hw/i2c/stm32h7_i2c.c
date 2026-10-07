/*
 * STM32H7 I2C controller (I2C v2, RM0399 chapter 52), master mode, on a
 * QEMU I2C bus named by the bus-name property (default "i2c") that devices
 * attach to (-device <type>,bus=<bus-name>,address=<addr>).
 *
 * Modeled: 7-bit addressing; START with NBYTES/RELOAD/AUTOEND; write (TXIS,
 * TXDR) and read (RXNE, RXDR); TC/TCR; NACK from an absent device or a
 * device that refuses a byte, followed by an automatic STOP (NACKF, STOPF);
 * STOP; the event and error interrupts. Each byte completes at once.
 *
 * Not modeled: slave mode, 10-bit addressing, SMBus PEC/alert, timeouts,
 * DMA, bus errors and arbitration loss.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"

#define TYPE_STM32H7_I2C "stm32h7-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7I2cState, STM32H7_I2C)

struct Stm32h7I2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq ev_irq, er_irq;
    I2CBus *bus;
    char *bus_name;

    uint32_t cr1, cr2, oar1, oar2, timingr, timeoutr, isr;
    uint8_t rxdr;
    uint32_t left;          /* bytes left in this NBYTES chunk */
    bool active, reading;
};

#define CR1_PE     BIT(0)
#define CR1_TXIE   BIT(1)
#define CR1_RXIE   BIT(2)
#define CR1_NACKIE BIT(4)
#define CR1_STOPIE BIT(5)
#define CR1_TCIE   BIT(6)
#define CR1_ERRIE  BIT(7)

#define CR2_RD_WRN  BIT(10)
#define CR2_START   BIT(13)
#define CR2_STOP    BIT(14)
#define CR2_RELOAD  BIT(24)
#define CR2_AUTOEND BIT(25)
#define CR2_NBYTES(v) (((v) >> 16) & 0xFF)

#define ISR_TXE   BIT(0)
#define ISR_TXIS  BIT(1)
#define ISR_RXNE  BIT(2)
#define ISR_NACKF BIT(4)
#define ISR_STOPF BIT(5)
#define ISR_TC    BIT(6)
#define ISR_TCR   BIT(7)
#define ISR_BERR  BIT(8)
#define ISR_ARLO  BIT(9)
#define ISR_OVR   BIT(10)
#define ISR_BUSY  BIT(15)
#define ISR_DIR   BIT(16)

static void update_irq(Stm32h7I2cState *s)
{
    uint32_t i = s->isr, c = s->cr1;
    bool ev = ((c & CR1_TXIE) && (i & ISR_TXIS)) ||
              ((c & CR1_RXIE) && (i & ISR_RXNE)) ||
              ((c & CR1_NACKIE) && (i & ISR_NACKF)) ||
              ((c & CR1_STOPIE) && (i & ISR_STOPF)) ||
              ((c & CR1_TCIE) && (i & (ISR_TC | ISR_TCR)));
    bool er = (c & CR1_ERRIE) && (i & (ISR_BERR | ISR_ARLO | ISR_OVR));

    qemu_set_irq(s->ev_irq, ev && (c & CR1_PE));
    qemu_set_irq(s->er_irq, er && (c & CR1_PE));
}

static void do_stop(Stm32h7I2cState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->active = false;
    s->left = 0;
    s->isr &= ~(ISR_BUSY | ISR_TXIS | ISR_DIR);
    s->isr |= ISR_STOPF | ISR_TXE;
    s->cr2 &= ~CR2_STOP;
}

static void nack(Stm32h7I2cState *s)
{
    s->isr |= ISR_NACKF;
    do_stop(s);             /* a NACK is followed by an automatic STOP */
}

/* All bytes of the current NBYTES chunk are done */
static void chunk_done(Stm32h7I2cState *s)
{
    if (s->cr2 & CR2_RELOAD) {
        s->isr |= ISR_TCR;
    } else if (s->cr2 & CR2_AUTOEND) {
        do_stop(s);
    } else {
        s->isr |= ISR_TC;
    }
}

static void read_next(Stm32h7I2cState *s)
{
    if (s->active && s->reading && s->left && !(s->isr & ISR_RXNE)) {
        s->rxdr = i2c_recv(s->bus);
        s->isr |= ISR_RXNE;
        s->left--;
    }
}

static void begin_chunk(Stm32h7I2cState *s)
{
    s->left = CR2_NBYTES(s->cr2);
    s->isr &= ~(ISR_TC | ISR_TCR);
    if (s->reading) {
        if (s->left) {
            read_next(s);
        } else {
            chunk_done(s);
        }
    } else if (s->left) {
        s->isr |= ISR_TXIS;
    } else {
        chunk_done(s);
    }
}

static void do_start(Stm32h7I2cState *s)
{
    uint8_t addr = (s->cr2 >> 1) & 0x7F;

    s->cr2 &= ~CR2_START;
    s->reading = s->cr2 & CR2_RD_WRN;
    s->isr &= ~(ISR_NACKF | ISR_STOPF | ISR_TC | ISR_TCR | ISR_RXNE);
    s->isr |= ISR_BUSY;
    s->isr = (s->isr & ~ISR_DIR) | (s->reading ? ISR_DIR : 0);
    /* a repeated start continues the current transaction */
    if (i2c_start_transfer(s->bus, addr, s->reading)) {
        s->active = false;
        nack(s);
        return;
    }
    s->active = true;
    begin_chunk(s);
}

static uint64_t i2c_read_reg(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7I2cState *s = opaque;
    uint64_t v = 0;

    switch (off) {
    case 0x00: v = s->cr1; break;
    case 0x04: v = s->cr2; break;
    case 0x08: v = s->oar1; break;
    case 0x0C: v = s->oar2; break;
    case 0x10: v = s->timingr; break;
    case 0x14: v = s->timeoutr; break;
    case 0x18: v = s->isr; break;
    case 0x24:
        v = s->rxdr;
        s->isr &= ~ISR_RXNE;
        if (s->left) {
            read_next(s);
        } else if (s->active) {
            chunk_done(s);
        }
        break;
    }
    update_irq(s);
    return v;
}

static void i2c_write_reg(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7I2cState *s = opaque;

    switch (off) {
    case 0x00:
        s->cr1 = v;
        if (!(v & CR1_PE)) {        /* software reset */
            if (s->active) {
                i2c_end_transfer(s->bus);
            }
            s->active = false;
            s->isr = ISR_TXE;
        }
        break;
    case 0x04:
        s->cr2 = v;
        if ((v & CR2_START) && (s->cr1 & CR1_PE)) {
            do_start(s);
        } else if (v & CR2_STOP) {
            do_stop(s);
        } else if (s->active && (s->isr & ISR_TCR)) {
            begin_chunk(s);         /* NBYTES reloaded */
        }
        break;
    case 0x08: s->oar1 = v; break;
    case 0x0C: s->oar2 = v; break;
    case 0x10: s->timingr = v; break;
    case 0x14: s->timeoutr = v; break;
    case 0x18:                      /* TXE can be set to flush TXDR */
        s->isr |= v & ISR_TXE;
        break;
    case 0x1C:                      /* ICR */
        s->isr &= ~(v & 0x3F38);
        break;
    case 0x28:
        if (s->active && !s->reading && s->left) {
            s->isr &= ~ISR_TXIS;
            if (i2c_send(s->bus, v)) {
                nack(s);
                break;
            }
            if (--s->left) {
                s->isr |= ISR_TXIS;
            } else {
                chunk_done(s);
            }
        }
        break;
    }
    update_irq(s);
}

static const MemoryRegionOps i2c_ops = {
    .read = i2c_read_reg,
    .write = i2c_write_reg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void i2c_reset(DeviceState *dev)
{
    Stm32h7I2cState *s = STM32H7_I2C(dev);

    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->cr1 = s->cr2 = s->oar1 = s->oar2 = s->timingr = s->timeoutr = 0;
    s->isr = ISR_TXE;
    s->active = false;
    s->left = 0;
}

static void i2c_init(Object *obj)
{
    Stm32h7I2cState *s = STM32H7_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &i2c_ops, s, "stm32h7-i2c", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->ev_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->er_irq);
}

static void i2c_realize(DeviceState *dev, Error **errp)
{
    Stm32h7I2cState *s = STM32H7_I2C(dev);

    s->bus = i2c_init_bus(dev, s->bus_name ? s->bus_name : "i2c");
}

static const Property i2c_props[] = {
    DEFINE_PROP_STRING("bus-name", Stm32h7I2cState, bus_name),
};

static void i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = i2c_realize;
    device_class_set_legacy_reset(dc, i2c_reset);
    device_class_set_props(dc, i2c_props);
}

static const TypeInfo i2c_info = {
    .name = TYPE_STM32H7_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7I2cState),
    .instance_init = i2c_init,
    .class_init = i2c_class_init,
};

static void i2c_register(void)
{
    type_register_static(&i2c_info);
}
type_init(i2c_register)
