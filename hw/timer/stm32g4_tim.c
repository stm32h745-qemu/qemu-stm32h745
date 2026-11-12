/*
 * STM32G4 timer (RM0440 chapters 28-31), counter core: up counting with
 * prescaler, auto-reload with optional preload, update event (interrupt,
 * DMA request, UG), one-pulse mode, and the master mode output (TRGO) that
 * triggers ADC conversions. The counter value is derived from the virtual
 * clock, and update events are scheduled at their exact tick time, so the
 * timing follows the timer clock the firmware configured and is
 * deterministic under -icount.
 *
 * Not modeled: capture/compare (CCxIF never set, no pin outputs), down and
 * center-aligned counting, slave mode, break, encoder.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/timer/stm32g4_tim.h"
#include "migration/vmstate.h"

#define CR1   0x00
#define CR2   0x04
#define SMCR  0x08
#define DIER  0x0C
#define SR    0x10
#define EGR   0x14
#define CCER  0x20
#define CNT   0x24
#define PSC   0x28
#define ARR   0x2C
#define RCR   0x30

#define CR1_CEN  BIT(0)
#define CR1_UDIS BIT(1)
#define CR1_URS  BIT(2)
#define CR1_OPM  BIT(3)
#define CR1_DIR  BIT(4)
#define CR1_ARPE BIT(7)
#define SR_UIF   BIT(0)
#define DIER_UIE BIT(0)
#define DIER_UDE BIT(8)
#define EGR_UG   BIT(0)

#define R(s, off) ((s)->regs[(off) / 4])

static uint64_t timclk(Stm32g4TimState *s)
{
    return s->clk_hz ? s->clk_hz(s->clk_opaque) : 0;
}

/* nanoseconds (rounded up) taken by n counter ticks */
static int64_t ticks_to_ns(Stm32g4TimState *s, uint64_t n)
{
    unsigned __int128 num = (unsigned __int128)n * NANOSECONDS_PER_SECOND *
                            (s->psc_active + 1ull);
    uint64_t hz = timclk(s);

    if (!hz) {
        return INT64_MAX / 2;
    }
    return (int64_t)((num + hz - 1) / hz);
}

static uint64_t ns_to_ticks(Stm32g4TimState *s, int64_t ns)
{
    unsigned __int128 num = (unsigned __int128)ns * timclk(s);

    return (uint64_t)(num / ((unsigned __int128)NANOSECONDS_PER_SECOND *
                             (s->psc_active + 1ull)));
}

static uint32_t cnt_mask(Stm32g4TimState *s)
{
    return s->width == 32 ? 0xFFFFFFFF : 0xFFFF;
}

/* Counter value now */
static uint32_t cnt_now(Stm32g4TimState *s)
{
    uint64_t period = (uint64_t)s->arr_active + 1;
    uint64_t t;

    if (!s->running) {
        return s->epoch_cnt;
    }
    t = ns_to_ticks(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->epoch_t);
    return (s->epoch_cnt + t) % period;
}

/* Move the epoch to the last whole tick so timing parameters can change */
static void rebase(Stm32g4TimState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t period = (uint64_t)s->arr_active + 1;
    uint64_t t;

    if (!s->running) {
        return;
    }
    t = ns_to_ticks(s, now - s->epoch_t);
    s->epoch_cnt = (s->epoch_cnt + t) % period;
    s->epoch_t += ticks_to_ns(s, t);
}

static void update_irq(Stm32g4TimState *s)
{
    bool l = (R(s, SR) & R(s, DIER) & SR_UIF);

    qemu_set_irq(s->irq[0], l);
}

static void schedule(Stm32g4TimState *s)
{
    uint64_t period = (uint64_t)s->arr_active + 1;
    uint64_t n;
    int64_t when, now;

    if (!s->running || !timclk(s)) {
        timer_del(s->timer);
        return;
    }
    n = period - s->epoch_cnt;
    when = s->epoch_t + ticks_to_ns(s, n);
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (when <= now) {
        when = now + 1;
    }
    timer_mod(s->timer, when);
}

static void apply_preload(Stm32g4TimState *s)
{
    s->arr_active = s->arr_shadow;
    s->psc_active = s->psc_shadow;
}

static void trgo_pulse(Stm32g4TimState *s)
{
    qemu_irq_pulse(s->trgo);
}

static void update_event(Stm32g4TimState *s, bool from_counter)
{
    if (!(R(s, CR1) & CR1_UDIS)) {
        R(s, SR) |= SR_UIF;
        update_irq(s);
        if (R(s, DIER) & DIER_UDE) {
            qemu_irq_pulse(s->dma_update);
        }
    }
    if (((R(s, CR2) >> 4) & 7) == 2 || (!from_counter && ((R(s, CR2) >> 4) & 7) == 0)) {
        trgo_pulse(s);
    }
}

static void tim_tick(void *opaque)
{
    Stm32g4TimState *s = opaque;
    uint64_t period = (uint64_t)s->arr_active + 1;

    /* the counter wrapped at exactly this tick: the new period starts there */
    s->epoch_t += ticks_to_ns(s, period - s->epoch_cnt);
    s->epoch_cnt = 0;
    apply_preload(s);
    update_event(s, true);
    if (R(s, CR1) & CR1_OPM) {
        R(s, CR1) &= ~CR1_CEN;
        s->running = false;
        timer_del(s->timer);
        return;
    }
    schedule(s);
}

static void set_running(Stm32g4TimState *s, bool on)
{
    if (on == s->running) {
        return;
    }
    if (on) {
        s->running = true;
        s->epoch_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (s->epoch_cnt > s->arr_active) {
            s->epoch_cnt = 0;
        }
        if (((R(s, CR2) >> 4) & 7) == 1) {
            qemu_irq_pulse(s->trgo);        /* MMS = enable: TRGO follows CEN */
        }
        schedule(s);
    } else {
        s->epoch_cnt = cnt_now(s);
        s->running = false;
        timer_del(s->timer);
    }
}

static uint64_t tim_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4TimState *s = opaque;

    switch (off) {
    case CNT: return cnt_now(s);
    case PSC: return s->psc_shadow;
    case ARR: return s->arr_shadow;
    }
    return off < sizeof(s->regs) ? s->regs[off / 4] : 0;
}

static void tim_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4TimState *s = opaque;

    switch (off) {
    case CR1:
        R(s, CR1) = val & 0x3FF;
        if (!(val & CR1_ARPE)) {
            rebase(s);
            s->arr_active = s->arr_shadow;
            schedule(s);
        }
        set_running(s, val & CR1_CEN);
        return;
    case CR2:
    case SMCR:
    case DIER:
        R(s, off) = val;
        update_irq(s);
        return;
    case SR:
        R(s, SR) &= val | ~0x1FFFFu;        /* rc_w0 */
        update_irq(s);
        return;
    case EGR:
        if (val & EGR_UG) {
            rebase(s);
            apply_preload(s);
            s->epoch_cnt = 0;
            s->epoch_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            if (R(s, CR1) & CR1_URS) {
                /* only counter overflow sets UIF; UG just reloads */
                if (((R(s, CR2) >> 4) & 7) == 0) {
                    trgo_pulse(s);
                }
            } else {
                update_event(s, false);
            }
            schedule(s);
        }
        if (val & ~EGR_UG & 0x1F) {
            qemu_log_mask(LOG_UNIMP, "stm32g4-tim: capture/compare events not modeled\n");
        }
        return;
    case CNT:
        rebase(s);
        s->epoch_cnt = val & cnt_mask(s);
        s->epoch_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        schedule(s);
        return;
    case PSC:
        s->psc_shadow = val & 0xFFFF;       /* takes effect at the next update event */
        return;
    case ARR:
        s->arr_shadow = val & cnt_mask(s);
        if (!(R(s, CR1) & CR1_ARPE)) {
            rebase(s);
            s->arr_active = s->arr_shadow;
            if (s->epoch_cnt > s->arr_active) {
                s->epoch_cnt = 0;
            }
            schedule(s);
        }
        return;
    case CCER:
        if ((val & 0x1111) && !s->cc_warned) {
            qemu_log_mask(LOG_UNIMP, "stm32g4-tim: capture/compare outputs not modeled\n");
            s->cc_warned = true;
        }
        break;
    }
    if (off < sizeof(s->regs)) {
        s->regs[off / 4] = val;
    }
}

static const MemoryRegionOps tim_ops = {
    .read = tim_read,
    .write = tim_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void tim_reset(DeviceState *dev)
{
    Stm32g4TimState *s = STM32G4_TIM(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    s->running = false;
    s->epoch_cnt = 0;
    s->psc_shadow = s->psc_active = 0;
    s->arr_shadow = s->arr_active = cnt_mask(s);
    update_irq(s);
}

static void tim_realize(DeviceState *dev, Error **errp)
{
    Stm32g4TimState *s = STM32G4_TIM(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tim_tick, s);
}

static void tim_init(Object *obj)
{
    Stm32g4TimState *s = STM32G4_TIM(obj);

    memory_region_init_io(&s->iomem, obj, &tim_ops, s, TYPE_STM32G4_TIM, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (int i = 0; i < 4; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), &s->trgo, "trgo", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dma_update, "dma-update", 1);
}

static const Property tim_props[] = {
    DEFINE_PROP_UINT32("width", Stm32g4TimState, width, 16),
};

static const VMStateDescription vmstate_tim = {
    .name = TYPE_STM32G4_TIM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Stm32g4TimState, 0x400 / 4),
        VMSTATE_UINT32(arr_shadow, Stm32g4TimState),
        VMSTATE_UINT32(psc_shadow, Stm32g4TimState),
        VMSTATE_UINT32(arr_active, Stm32g4TimState),
        VMSTATE_UINT32(psc_active, Stm32g4TimState),
        VMSTATE_BOOL(running, Stm32g4TimState),
        VMSTATE_INT64(epoch_t, Stm32g4TimState),
        VMSTATE_UINT32(epoch_cnt, Stm32g4TimState),
        VMSTATE_END_OF_LIST()
    }
};

static void tim_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tim_realize;
    device_class_set_legacy_reset(dc, tim_reset);
    device_class_set_props(dc, tim_props);
    dc->vmsd = &vmstate_tim;
}

static const TypeInfo tim_info = {
    .name = TYPE_STM32G4_TIM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4TimState),
    .instance_init = tim_init,
    .class_init = tim_class_init,
};

static void tim_register(void)
{
    type_register_static(&tim_info);
}
type_init(tim_register)
