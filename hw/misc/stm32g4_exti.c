/*
 * STM32G4 EXTI (RM0440 chapter 17) for the GPIO lines 0-15, with SYSCFG's
 * EXTICR port selection. Pad level changes of the GPIO ports arrive on the
 * "gpio" inputs (port * 16 + pin); the port selected for a line by SYSCFG
 * generates its edges. Rising/falling trigger selection, pending register,
 * software interrupt register and the NVIC lines (EXTI0-4, EXTI9_5,
 * EXTI15_10) are modeled. Lines 16 and up (peripheral events) are not wired.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/stm32g4_exti.h"
#include "migration/vmstate.h"

#define EXTI_IMR1   0x00
#define EXTI_EMR1   0x04
#define EXTI_RTSR1  0x08
#define EXTI_FTSR1  0x0C
#define EXTI_SWIER1 0x10
#define EXTI_PR1    0x14
#define EXTI_IMR2   0x20
#define EXTI_EMR2   0x24
#define EXTI_RTSR2  0x28
#define EXTI_FTSR2  0x2C
#define EXTI_SWIER2 0x30
#define EXTI_PR2    0x34

#define SYSCFG_EXTICR1 0x08

static void update_irqs(Stm32g4ExtiState *s)
{
    uint32_t act = s->pr[0] & s->imr[0];
    static const uint32_t groups[STM32G4_EXTI_IRQS] = {
        BIT(0), BIT(1), BIT(2), BIT(3), BIT(4), 0x3E0, 0xFC00,
    };

    for (int i = 0; i < STM32G4_EXTI_IRQS; i++) {
        qemu_set_irq(s->irq[i], !!(act & groups[i]));
    }
}

static int line_port(Stm32g4ExtiState *s, int line)
{
    return (s->syscfg[SYSCFG_EXTICR1 / 4 + line / 4] >> (4 * (line % 4))) & 0xF;
}

static void line_changed(Stm32g4ExtiState *s, int line, bool level)
{
    bool old = (s->line_level >> line) & 1;

    if (old == level) {
        return;
    }
    s->line_level = (s->line_level & ~BIT(line)) | (level << line);
    if ((level && (s->rtsr[0] & BIT(line))) || (!level && (s->ftsr[0] & BIT(line)))) {
        s->pr[0] |= BIT(line);
    }
}

static void gpio_in(void *opaque, int n, int level)
{
    Stm32g4ExtiState *s = opaque;
    int port = n / 16, pin = n % 16;

    s->port_level[port] = (s->port_level[port] & ~BIT(pin)) | ((!!level) << pin);
    if (line_port(s, pin) == port) {
        line_changed(s, pin, level);
        update_irqs(s);
    }
}

static void resync_lines(Stm32g4ExtiState *s)
{
    for (int line = 0; line < 16; line++) {
        int port = line_port(s, line);
        bool lvl = port < STM32G4_EXTI_PORTS ? (s->port_level[port] >> line) & 1 : 0;

        line_changed(s, line, lvl);
    }
    update_irqs(s);
}

static uint64_t exti_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4ExtiState *s = opaque;

    switch (off) {
    case EXTI_RTSR1:  return s->rtsr[0];
    case EXTI_FTSR1:  return s->ftsr[0];
    case EXTI_SWIER1: return s->swier[0];
    case EXTI_PR1:    return s->pr[0];
    case EXTI_RTSR2:  return s->rtsr[1];
    case EXTI_FTSR2:  return s->ftsr[1];
    case EXTI_SWIER2: return s->swier[1];
    case EXTI_PR2:    return s->pr[1];
    case EXTI_IMR1:   return s->imr[0];
    case EXTI_EMR1:   return s->emr[0];
    case EXTI_IMR2:   return s->imr[1];
    case EXTI_EMR2:   return s->emr[1];
    }
    return 0;
}

static void exti_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4ExtiState *s = opaque;

    switch (off) {
    case EXTI_RTSR1:  s->rtsr[0] = val; break;
    case EXTI_FTSR1:  s->ftsr[0] = val; break;
    case EXTI_SWIER1:
        s->swier[0] = val;
        s->pr[0] |= val;
        break;
    case EXTI_PR1:
        s->pr[0] &= ~val;
        s->swier[0] &= ~val;
        break;
    case EXTI_RTSR2:  s->rtsr[1] = val; break;
    case EXTI_FTSR2:  s->ftsr[1] = val; break;
    case EXTI_SWIER2: s->swier[1] = val; s->pr[1] |= val; break;
    case EXTI_PR2:    s->pr[1] &= ~val; s->swier[1] &= ~val; break;
    case EXTI_IMR1:   s->imr[0] = val; break;
    case EXTI_EMR1:   s->emr[0] = val; break;
    case EXTI_IMR2:   s->imr[1] = val; break;
    case EXTI_EMR2:   s->emr[1] = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "stm32g4-exti: write to 0x%x ignored\n", (unsigned)off);
        return;
    }
    update_irqs(s);
}

static uint64_t syscfg_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4ExtiState *s = opaque;

    return off < sizeof(s->syscfg) ? s->syscfg[off / 4] : 0;
}

static void syscfg_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4ExtiState *s = opaque;

    if (off >= sizeof(s->syscfg)) {
        return;
    }
    if (off >= SYSCFG_EXTICR1 && off < SYSCFG_EXTICR1 + 16) {
        val &= 0xFFFF;
    }
    s->syscfg[off / 4] = val;
    if (off >= SYSCFG_EXTICR1 && off < SYSCFG_EXTICR1 + 16) {
        resync_lines(s);
    }
}

#define OPS(name)                                                       \
    static const MemoryRegionOps name##_ops = {                         \
        .read = name##_read,                                            \
        .write = name##_write,                                          \
        .endianness = DEVICE_LITTLE_ENDIAN,                             \
        .impl = { .min_access_size = 4, .max_access_size = 4 },         \
        .valid = { .min_access_size = 1, .max_access_size = 4 },        \
    };
OPS(exti)
OPS(syscfg)

static void exti_reset(DeviceState *dev)
{
    Stm32g4ExtiState *s = STM32G4_EXTI(dev);

    memset(s->rtsr, 0, sizeof(s->rtsr));
    memset(s->ftsr, 0, sizeof(s->ftsr));
    memset(s->swier, 0, sizeof(s->swier));
    memset(s->pr, 0, sizeof(s->pr));
    memset(s->emr, 0, sizeof(s->emr));
    s->imr[0] = 0xFF820000;     /* RM0440: direct event lines are unmasked */
    s->imr[1] = 0x00000000;
    memset(s->syscfg, 0, sizeof(s->syscfg));
    s->syscfg[1] = 0x7C000001;  /* CFGR1 */
    /* The ports keep their pad levels; lines follow port A until firmware
     * selects another one */
    s->line_level = 0;
    for (int line = 0; line < 16; line++) {
        s->line_level |= ((s->port_level[0] >> line) & 1) << line;
    }
    update_irqs(s);
}

static void exti_init(Object *obj)
{
    Stm32g4ExtiState *s = STM32G4_EXTI(obj);

    memory_region_init_io(&s->exti_mmio, obj, &exti_ops, s, "stm32g4-exti", 0x400);
    memory_region_init_io(&s->syscfg_mmio, obj, &syscfg_ops, s, "stm32g4-syscfg", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->exti_mmio);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->syscfg_mmio);
    for (int i = 0; i < STM32G4_EXTI_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_in_named(DEVICE(obj), gpio_in, "gpio", STM32G4_EXTI_PORTS * 16);
}

static const VMStateDescription vmstate_exti = {
    .name = TYPE_STM32G4_EXTI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(rtsr, Stm32g4ExtiState, 2),
        VMSTATE_UINT32_ARRAY(ftsr, Stm32g4ExtiState, 2),
        VMSTATE_UINT32_ARRAY(swier, Stm32g4ExtiState, 2),
        VMSTATE_UINT32_ARRAY(pr, Stm32g4ExtiState, 2),
        VMSTATE_UINT32_ARRAY(imr, Stm32g4ExtiState, 2),
        VMSTATE_UINT32_ARRAY(emr, Stm32g4ExtiState, 2),
        VMSTATE_UINT32_ARRAY(syscfg, Stm32g4ExtiState, 0x40 / 4),
        VMSTATE_UINT16_ARRAY(port_level, Stm32g4ExtiState, STM32G4_EXTI_PORTS),
        VMSTATE_UINT16(line_level, Stm32g4ExtiState),
        VMSTATE_END_OF_LIST()
    }
};

static void exti_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, exti_reset);
    dc->vmsd = &vmstate_exti;
}

static const TypeInfo exti_info = {
    .name = TYPE_STM32G4_EXTI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4ExtiState),
    .instance_init = exti_init,
    .class_init = exti_class_init,
};

static void exti_register(void)
{
    type_register_static(&exti_info);
}
type_init(exti_register)
