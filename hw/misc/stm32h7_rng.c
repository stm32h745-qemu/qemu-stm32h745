/*
 * STM32H7 true random number generator (RM0399 chapter 34).
 *
 * Data is always ready while the RNG is enabled (DRDY reads 1) and comes from
 * QEMU's guest random source (-seed makes it reproducible). No clock or seed
 * errors are ever flagged. The interrupt is DRDY while IE and RNGEN are set.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/guest-random.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32H7_RNG "stm32h7-rng"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7RngState, STM32H7_RNG)

struct Stm32h7RngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t cr, htcr;
};

#define CR_RNGEN   BIT(2)
#define CR_IE      BIT(3)
#define CR_CONDRST BIT(30)
#define SR_DRDY    BIT(0)

static bool ready(Stm32h7RngState *s)
{
    return (s->cr & CR_RNGEN) && !(s->cr & CR_CONDRST);
}

static void update_irq(Stm32h7RngState *s)
{
    qemu_set_irq(s->irq, (s->cr & CR_IE) && ready(s));
}

static uint64_t rng_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7RngState *s = opaque;
    uint32_t v = 0;

    switch (off) {
    case 0x00: return s->cr;
    case 0x04: return ready(s) ? SR_DRDY : 0;
    case 0x08:
        if (ready(s)) {
            qemu_guest_getrandom_nofail(&v, sizeof(v));
        }
        return v;
    case 0x10: return s->htcr;
    }
    return 0;
}

static void rng_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7RngState *s = opaque;

    switch (off) {
    case 0x00: s->cr = v; break;
    case 0x04: break;           /* error flags: never set */
    case 0x10: s->htcr = v; break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-rng: write 0x%" HWADDR_PRIx "\n", off);
    }
    update_irq(s);
}

static const MemoryRegionOps rng_ops = {
    .read = rng_read,
    .write = rng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rng_reset(DeviceState *dev)
{
    Stm32h7RngState *s = STM32H7_RNG(dev);

    s->cr = 0x00800000;
    s->htcr = 0x000072AC;
    update_irq(s);
}

static void rng_init(Object *obj)
{
    Stm32h7RngState *s = STM32H7_RNG(obj);

    memory_region_init_io(&s->iomem, obj, &rng_ops, s, "stm32h7-rng", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void rng_class_init(ObjectClass *klass, void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), rng_reset);
}

static const TypeInfo rng_info = {
    .name = TYPE_STM32H7_RNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7RngState),
    .instance_init = rng_init,
    .class_init = rng_class_init,
};

static void rng_register(void)
{
    type_register_static(&rng_info);
}
type_init(rng_register)
