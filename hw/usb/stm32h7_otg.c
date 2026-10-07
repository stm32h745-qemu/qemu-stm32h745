/*
 * STM32H7 USB OTG controller (Synopsys DWC2, RM0399 chapter 57) with no
 * cable attached.
 *
 * Firmware can initialize the core in device mode and start it: the core
 * and FIFO resets complete at once (AHBIDL reads 1), the mode switch takes
 * effect immediately (GINTSTS.CMOD follows GUSBCFG.FHMOD), and the register
 * file keeps what is written. No host ever connects, so no bus reset,
 * enumeration or transfer interrupts happen: the state of a board in the
 * field with nothing on its USB port.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32H7_OTG "stm32h7-otg"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7OtgState, STM32H7_OTG)

#define OTG_SIZE   0x40000
#define REGS_SIZE  0x1000      /* core, host and device registers */

#define GUSBCFG   0x0C
#define GRSTCTL   0x10
#define GINTSTS   0x14
#define GSNPSID   0x40
#define GHWCFG2   0x48
#define CID       0x3C

#define GUSBCFG_FHMOD   BIT(29)
#define GRSTCTL_CSRST   BIT(0)
#define GRSTCTL_HSRST   BIT(1)
#define GRSTCTL_RXFFLSH BIT(4)
#define GRSTCTL_TXFFLSH BIT(5)
#define GRSTCTL_AHBIDL  BIT(31)

struct Stm32h7OtgState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[REGS_SIZE / 4];
};

static uint64_t otg_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7OtgState *s = opaque;

    if (off >= REGS_SIZE) {
        return 0;               /* power/clock gating and the FIFO windows */
    }
    switch (off) {
    case GRSTCTL:
        return (s->regs[off / 4] & ~(GRSTCTL_CSRST | GRSTCTL_HSRST |
                                     GRSTCTL_RXFFLSH | GRSTCTL_TXFFLSH)) |
               GRSTCTL_AHBIDL;
    case GINTSTS:
        /* CMOD: 1 = host mode; no interrupt sources ever pend */
        return (s->regs[GUSBCFG / 4] & GUSBCFG_FHMOD) ? 1 : 0;
    case GSNPSID:
        return 0x4F54330A;
    case GHWCFG2:
        return 0x229DCD20;
    }
    return s->regs[off / 4];
}

static void otg_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7OtgState *s = opaque;

    if (off < REGS_SIZE && off != GINTSTS && off != GSNPSID) {
        s->regs[off / 4] = v;
    }
}

static const MemoryRegionOps otg_ops = {
    .read = otg_read,
    .write = otg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void otg_reset(DeviceState *dev)
{
    Stm32h7OtgState *s = STM32H7_OTG(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00 / 4] = 0x00000800;     /* GOTGCTL: B-session valid off */
    s->regs[GUSBCFG / 4] = 0x00001440;
    s->regs[CID / 4] = 0x00001200;
}

static void otg_init(Object *obj)
{
    Stm32h7OtgState *s = STM32H7_OTG(obj);

    memory_region_init_io(&s->iomem, obj, &otg_ops, s, "stm32h7-otg", OTG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void otg_class_init(ObjectClass *klass, void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), otg_reset);
}

static const TypeInfo otg_info = {
    .name = TYPE_STM32H7_OTG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7OtgState),
    .instance_init = otg_init,
    .class_init = otg_class_init,
};

static void otg_register(void)
{
    type_register_static(&otg_info);
}
type_init(otg_register)
