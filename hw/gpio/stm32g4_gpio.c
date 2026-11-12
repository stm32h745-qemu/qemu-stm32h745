/*
 * STM32G4 GPIO port (RM0440 chapter 8).
 *
 * MODER/OTYPER/OSPEEDR/PUPDR/ODR/BSRR/BRR/AFR latch; IDR reads the pad level:
 * an output pin shows its output (an open-drain output driven high shows what
 * the outside or the pull resistor gives), an input pin shows the level
 * driven from outside, or the pull resistor, or 0 if floating. The outside
 * world drives a pin through:
 *   - the "in" GPIO inputs (qemu_set_irq, used by other devices such as the
 *     AC source), level 0 or 1;
 *   - the "pinN" QOM properties, an integer: 0, 1, or -1 to release the pin
 *     (qom-set from QMP, or -global for the initial value).
 * Every change of the pad level is signalled on the "pad" GPIO outputs (the
 * EXTI model listens), and the read-only "pad" property gives all 16 levels
 * (for example to watch a relay output).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/stm32g4_gpio.h"
#include "migration/vmstate.h"
#include "trace.h"

#define MODER   0x00
#define OTYPER  0x04
#define OSPEEDR 0x08
#define PUPDR   0x0C
#define IDR     0x10
#define ODR     0x14
#define BSRR    0x18
#define LCKR    0x1C
#define AFRL    0x20
#define AFRH    0x24
#define BRR     0x28

static bool pin_pad(Stm32g4GpioState *s, int n)
{
    uint32_t mode = (s->moder >> (2 * n)) & 3;
    uint32_t pull = (s->pupdr >> (2 * n)) & 3;
    bool ext = (s->ext_driven >> n) & 1;
    bool ext_lvl = (s->ext_level >> n) & 1;
    bool pulled = pull == 1;        /* pull-up */

    if (mode == 1) {                /* general purpose output */
        bool od = (s->otyper >> n) & 1;
        bool o = (s->odr >> n) & 1;

        if (!od || !o) {
            return o;               /* push-pull, or open drain pulling low */
        }
        if (ext) {
            return ext_lvl;
        }
        return pulled;
    }
    return ext ? ext_lvl : pulled;
}

static void update_pads(Stm32g4GpioState *s)
{
    uint16_t now = 0;

    for (int n = 0; n < STM32G4_GPIO_PINS; n++) {
        now |= pin_pad(s, n) << n;
    }
    if (now != s->pad) {
        uint16_t diff = now ^ s->pad;

        s->pad = now;
        for (int n = 0; n < STM32G4_GPIO_PINS; n++) {
            if (diff & BIT(n)) {
                trace_stm32g4_gpio_pad(s->name ? s->name : "?", n, (now >> n) & 1);
                qemu_set_irq(s->pad_out[n], (now >> n) & 1);
            }
        }
    }
}

void stm32g4_gpio_set_input(Stm32g4GpioState *s, int pin, int level)
{
    if (pin < 0 || pin >= STM32G4_GPIO_PINS) {
        return;
    }
    if (level < 0) {
        s->ext_driven &= ~BIT(pin);
        s->ext_level &= ~BIT(pin);
    } else {
        s->ext_driven |= BIT(pin);
        s->ext_level = (s->ext_level & ~BIT(pin)) | ((level ? 1 : 0) << pin);
    }
    update_pads(s);
}

static void in_set(void *opaque, int n, int level)
{
    stm32g4_gpio_set_input(opaque, n, level ? 1 : 0);
}

static uint32_t idr_value(Stm32g4GpioState *s)
{
    uint32_t v = 0;

    for (int n = 0; n < STM32G4_GPIO_PINS; n++) {
        if (((s->moder >> (2 * n)) & 3) != 3) {     /* analog mode: reads 0 */
            v |= (uint32_t)((s->pad >> n) & 1) << n;
        }
    }
    return v;
}

static uint64_t gpio_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4GpioState *s = opaque;

    switch (off) {
    case MODER:   return s->moder;
    case OTYPER:  return s->otyper;
    case OSPEEDR: return s->ospeedr;
    case PUPDR:   return s->pupdr;
    case IDR:     return idr_value(s);
    case ODR:     return s->odr;
    case LCKR:    return s->lckr;
    case AFRL:    return s->afrl;
    case AFRH:    return s->afrh;
    }
    return 0;
}

static void gpio_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4GpioState *s = opaque;

    switch (off) {
    case MODER:   s->moder = val; break;
    case OTYPER:  s->otyper = val & 0xFFFF; break;
    case OSPEEDR: s->ospeedr = val; break;
    case PUPDR:   s->pupdr = val; break;
    case ODR:     s->odr = val & 0xFFFF; break;
    case BSRR:
        s->odr |= val & 0xFFFF;
        s->odr &= ~((val >> 16) & 0xFFFF);
        break;
    case BRR:     s->odr &= ~(val & 0xFFFF); break;
    case LCKR:    s->lckr = val & 0x1FFFF; break;
    case AFRL:    s->afrl = val; break;
    case AFRH:    s->afrh = val; break;
    case IDR:     return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32g4-gpio: bad write offset 0x%x\n", (unsigned)off);
        return;
    }
    update_pads(s);
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void gpio_reset(DeviceState *dev)
{
    Stm32g4GpioState *s = STM32G4_GPIO(dev);

    s->moder = s->mode_reset;
    s->ospeedr = s->ospeed_reset;
    s->pupdr = s->pupd_reset;
    s->otyper = s->odr = s->lckr = s->afrl = s->afrh = 0;
    update_pads(s);
}

static void pin_get(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    Stm32g4GpioState *s = STM32G4_GPIO(obj);
    int pin = (intptr_t)opaque;
    int64_t val = ((s->ext_driven >> pin) & 1) ? ((s->ext_level >> pin) & 1) : -1;

    visit_type_int(v, name, &val, errp);
}

static void pin_set(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    int64_t val;

    if (!visit_type_int(v, name, &val, errp)) {
        return;
    }
    if (val < -1 || val > 1) {
        error_setg(errp, "%s: value must be 0, 1 or -1 (release)", name);
        return;
    }
    stm32g4_gpio_set_input(STM32G4_GPIO(obj), (intptr_t)opaque, val);
}

static void pad_get(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    Stm32g4GpioState *s = STM32G4_GPIO(obj);
    uint32_t val = s->pad;

    visit_type_uint32(v, name, &val, errp);
}

static void gpio_init(Object *obj)
{
    Stm32g4GpioState *s = STM32G4_GPIO(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s, TYPE_STM32G4_GPIO, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(dev, in_set, "in", STM32G4_GPIO_PINS);
    qdev_init_gpio_out_named(dev, s->pad_out, "pad", STM32G4_GPIO_PINS);
    for (int n = 0; n < STM32G4_GPIO_PINS; n++) {
        g_autofree char *name = g_strdup_printf("pin%d", n);

        object_property_add(obj, name, "int", pin_get, pin_set, NULL, (void *)(intptr_t)n);
        object_property_set_description(obj, name,
                                        "external drive of the pin: 0, 1, or -1 (released)");
    }
    object_property_add(obj, "pad", "uint32", pad_get, NULL, NULL, NULL);
    object_property_set_description(obj, "pad", "pad levels of all 16 pins (read only)");
}

static const Property gpio_props[] = {
    DEFINE_PROP_STRING("name", Stm32g4GpioState, name),
    DEFINE_PROP_UINT32("mode-reset", Stm32g4GpioState, mode_reset, 0xFFFFFFFF),
    DEFINE_PROP_UINT32("ospeed-reset", Stm32g4GpioState, ospeed_reset, 0),
    DEFINE_PROP_UINT32("pupd-reset", Stm32g4GpioState, pupd_reset, 0),
};

static const VMStateDescription vmstate_gpio = {
    .name = TYPE_STM32G4_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(moder, Stm32g4GpioState),
        VMSTATE_UINT32(otyper, Stm32g4GpioState),
        VMSTATE_UINT32(ospeedr, Stm32g4GpioState),
        VMSTATE_UINT32(pupdr, Stm32g4GpioState),
        VMSTATE_UINT32(odr, Stm32g4GpioState),
        VMSTATE_UINT32(lckr, Stm32g4GpioState),
        VMSTATE_UINT32(afrl, Stm32g4GpioState),
        VMSTATE_UINT32(afrh, Stm32g4GpioState),
        VMSTATE_UINT16(ext_level, Stm32g4GpioState),
        VMSTATE_UINT16(ext_driven, Stm32g4GpioState),
        VMSTATE_UINT16(pad, Stm32g4GpioState),
        VMSTATE_END_OF_LIST()
    }
};

static void gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, gpio_reset);
    device_class_set_props(dc, gpio_props);
    dc->vmsd = &vmstate_gpio;
}

static const TypeInfo gpio_info = {
    .name = TYPE_STM32G4_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4GpioState),
    .instance_init = gpio_init,
    .class_init = gpio_class_init,
};

static void gpio_register(void)
{
    type_register_static(&gpio_info);
}
type_init(gpio_register)
