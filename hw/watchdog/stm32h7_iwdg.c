/*
 * STM32H7 independent watchdog (RM0399 chapter 48).
 *
 * Started by key 0xCCCC, refreshed by 0xAAAA; PR/RLR/WINR are writable after
 * 0x5555 and update at once (SR reads 0). The counter runs on the 32 kHz LSI
 * against QEMU's virtual clock; when it reaches 0 the -watchdog-action is
 * taken (reset by default). The window is not enforced.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "system/watchdog.h"
#include "qom/object.h"

#define TYPE_STM32H7_IWDG "stm32h7-iwdg"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7IwdgState, STM32H7_IWDG)

#define LSI_HZ 32000

struct Stm32h7IwdgState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *timer;
    uint32_t pr, rlr, winr;
    bool unlocked, running;
};

static void reload(Stm32h7IwdgState *s)
{
    uint64_t ticks = (uint64_t)(s->rlr + 1) * (4 << MIN(s->pr, 6));

    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        muldiv64(ticks, NANOSECONDS_PER_SECOND, LSI_HZ));
}

static void expired(void *opaque)
{
    qemu_log_mask(CPU_LOG_RESET, "stm32h7-iwdg: timeout\n");
    watchdog_perform_action();
}

static uint64_t iwdg_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7IwdgState *s = opaque;

    switch (off) {
    case 0x04: return s->pr;
    case 0x08: return s->rlr;
    case 0x0C: return 0;            /* updates complete at once */
    case 0x10: return s->winr;
    }
    return 0;
}

static void iwdg_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7IwdgState *s = opaque;

    switch (off) {
    case 0x00:
        switch (v & 0xFFFF) {
        case 0xCCCC: s->running = true; reload(s); break;
        case 0xAAAA: if (s->running) { reload(s); } break;
        case 0x5555: s->unlocked = true; break;
        default: s->unlocked = false;
        }
        break;
    case 0x04: if (s->unlocked) { s->pr = v & 7; } break;
    case 0x08: if (s->unlocked) { s->rlr = v & 0xFFF; } break;
    case 0x10: if (s->unlocked) { s->winr = v & 0xFFF; } break;
    }
}

static const MemoryRegionOps iwdg_ops = {
    .read = iwdg_read,
    .write = iwdg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

static void iwdg_reset(DeviceState *dev)
{
    Stm32h7IwdgState *s = STM32H7_IWDG(dev);

    timer_del(s->timer);
    s->pr = 0;
    s->rlr = 0xFFF;
    s->winr = 0xFFF;
    s->unlocked = s->running = false;
}

static void iwdg_init(Object *obj)
{
    Stm32h7IwdgState *s = STM32H7_IWDG(obj);

    memory_region_init_io(&s->iomem, obj, &iwdg_ops, s, "stm32h7-iwdg", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, expired, s);
}

static void iwdg_class_init(ObjectClass *klass, void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), iwdg_reset);
}

static const TypeInfo iwdg_info = {
    .name = TYPE_STM32H7_IWDG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7IwdgState),
    .instance_init = iwdg_init,
    .class_init = iwdg_class_init,
};

static void iwdg_register(void)
{
    type_register_static(&iwdg_info);
}
type_init(iwdg_register)
