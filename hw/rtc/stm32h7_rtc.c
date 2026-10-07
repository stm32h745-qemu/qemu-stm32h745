/*
 * STM32H7 real-time clock (RM0399 chapter 46): calendar, alarms A and B,
 * wakeup timer and 32 backup registers.
 *
 * The calendar starts from QEMU's RTC time (-rtc base=..., host UTC by
 * default) and runs on the RTC clock (-rtc clock=host|vm|rt). Firmware sets
 * it through INIT mode as on hardware. Alarm A/B raise the alarm interrupt
 * output (EXTI line 17 on hardware, wired straight to the NVIC here), the
 * wakeup timer the wakeup output. Shadow registers are always synchronized
 * (RSF reads 1); write protection is not enforced; the backup registers and
 * the calendar survive a guest reset (backup domain), as on hardware.
 *
 * Not modeled: timestamp, tamper, smooth calibration, the shift register,
 * daylight-saving bits, sub-second alarm masks (alarms match on whole
 * seconds).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "system/rtc.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_STM32H7_RTC "stm32h7-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7RtcState, STM32H7_RTC)

#define NUM_BKP 32

struct Stm32h7RtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq alarm_irq, wkup_irq;
    QEMUTimer *tick;            /* each second boundary: alarms */
    QEMUTimer *wut;             /* wakeup timer */

    int64_t base_ns;            /* RTC clock at base_sec */
    int64_t base_sec;           /* calendar seconds since 1970 at base_ns */
    uint32_t tr, dr;            /* held values in INIT mode */
    uint32_t cr, isr, prer, wutr, alrmar, alrmbr, calr, tampcr, or;
    uint32_t alrmassr, alrmbssr;
    uint32_t bkp[NUM_BKP];
    bool init_mode;
};

#define ISR_ALRAWF BIT(0)
#define ISR_ALRBWF BIT(1)
#define ISR_WUTWF  BIT(2)
#define ISR_INITS  BIT(4)
#define ISR_RSF    BIT(5)
#define ISR_INITF  BIT(6)
#define ISR_INIT   BIT(7)
#define ISR_ALRAF  BIT(8)
#define ISR_ALRBF  BIT(9)
#define ISR_WUTF   BIT(10)

#define CR_ALRAE  BIT(8)
#define CR_ALRBE  BIT(9)
#define CR_WUTE   BIT(10)
#define CR_ALRAIE BIT(12)
#define CR_ALRBIE BIT(13)
#define CR_WUTIE  BIT(14)

static int bcd(int v) { return ((v / 10) << 4) | (v % 10); }
static int unbcd(int v) { return (v >> 4) * 10 + (v & 0xF); }

static int64_t now_ns(void)
{
    return qemu_clock_get_ns(rtc_clock);
}

static int64_t cal_sec(Stm32h7RtcState *s)
{
    return s->base_sec + (now_ns() - s->base_ns) / NANOSECONDS_PER_SECOND;
}

static void sec_to_regs(int64_t sec, uint32_t *tr, uint32_t *dr)
{
    time_t t = sec;
    struct tm tm;

    gmtime_r(&t, &tm);
    *tr = bcd(tm.tm_hour) << 16 | bcd(tm.tm_min) << 8 | bcd(tm.tm_sec);
    *dr = bcd(tm.tm_year % 100) << 16 | ((tm.tm_wday ? tm.tm_wday : 7) << 13) |
          bcd(tm.tm_mon + 1) << 8 | bcd(tm.tm_mday);
}

static int64_t regs_to_sec(uint32_t tr, uint32_t dr)
{
    struct tm tm = {
        .tm_sec = unbcd(tr & 0x7F),
        .tm_min = unbcd((tr >> 8) & 0x7F),
        .tm_hour = unbcd((tr >> 16) & 0x3F),
        .tm_mday = unbcd(dr & 0x3F),
        .tm_mon = unbcd((dr >> 8) & 0x1F) - 1,
        .tm_year = unbcd((dr >> 16) & 0xFF) + 100,      /* 2000-2099 */
    };
    if ((tr & BIT(22)) && tm.tm_hour < 12) {            /* PM, 12-hour format */
        tm.tm_hour += 12;
    }
    return mktimegm(&tm);
}

static void update_irq(Stm32h7RtcState *s)
{
    qemu_set_irq(s->alarm_irq, ((s->cr & CR_ALRAIE) && (s->isr & ISR_ALRAF)) ||
                               ((s->cr & CR_ALRBIE) && (s->isr & ISR_ALRBF)));
    qemu_set_irq(s->wkup_irq, (s->cr & CR_WUTIE) && (s->isr & ISR_WUTF));
}

static bool alarm_match(uint32_t alrm, uint32_t tr, uint32_t dr)
{
    if (!(alrm & BIT(7)) && (alrm & 0x7F) != (tr & 0x7F)) {
        return false;
    }
    if (!(alrm & BIT(15)) && ((alrm >> 8) & 0x7F) != ((tr >> 8) & 0x7F)) {
        return false;
    }
    if (!(alrm & BIT(23)) && ((alrm >> 16) & 0x7F) != ((tr >> 16) & 0x7F)) {
        return false;
    }
    if (!(alrm & BIT(31))) {
        if (alrm & BIT(30)) {       /* weekday */
            if (((alrm >> 24) & 0xF) != ((dr >> 13) & 7)) {
                return false;
            }
        } else if (((alrm >> 24) & 0x3F) != (dr & 0x3F)) {
            return false;
        }
    }
    return true;
}

static void tick_arm(Stm32h7RtcState *s)
{
    int64_t n = now_ns();
    int64_t into = (n - s->base_ns) % NANOSECONDS_PER_SECOND;

    if (into < 0) {
        into += NANOSECONDS_PER_SECOND;
    }
    timer_mod(s->tick, n + NANOSECONDS_PER_SECOND - into);
}

static void tick_cb(void *opaque)
{
    Stm32h7RtcState *s = opaque;
    uint32_t tr, dr;

    if (!s->init_mode) {
        sec_to_regs(cal_sec(s), &tr, &dr);
        if ((s->cr & CR_ALRAE) && alarm_match(s->alrmar, tr, dr)) {
            s->isr |= ISR_ALRAF;
        }
        if ((s->cr & CR_ALRBE) && alarm_match(s->alrmbr, tr, dr)) {
            s->isr |= ISR_ALRBF;
        }
        update_irq(s);
    }
    tick_arm(s);
}

static int64_t wut_period_ns(Stm32h7RtcState *s)
{
    uint32_t sel = s->cr & 7;
    uint64_t ticks = (uint64_t)s->wutr + 1;

    if (sel < 4) {          /* RTCCLK (32.768 kHz) / 16, 8, 4, 2 */
        return muldiv64(ticks, NANOSECONDS_PER_SECOND * (16 >> sel), 32768);
    }
    if (sel >= 6) {
        ticks += 0x10000;
    }
    return ticks * NANOSECONDS_PER_SECOND;     /* ck_spre = 1 Hz */
}

static void wut_cb(void *opaque)
{
    Stm32h7RtcState *s = opaque;

    if (s->cr & CR_WUTE) {
        s->isr |= ISR_WUTF;
        update_irq(s);
        timer_mod(s->wut, now_ns() + wut_period_ns(s));
    }
}

static uint32_t ssr(Stm32h7RtcState *s)
{
    uint32_t prediv_s = s->prer & 0x7FFF;
    int64_t into = (now_ns() - s->base_ns) % NANOSECONDS_PER_SECOND;

    if (into < 0) {
        into += NANOSECONDS_PER_SECOND;
    }
    return prediv_s - muldiv64(into, prediv_s + 1, NANOSECONDS_PER_SECOND);
}

static uint64_t rtc_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7RtcState *s = opaque;
    uint32_t tr, dr;

    if (off >= 0x50 && off < 0x50 + 4 * NUM_BKP) {
        return s->bkp[(off - 0x50) / 4];
    }
    switch (off) {
    case 0x00: case 0x04:
        if (s->init_mode) {
            return off ? s->dr : s->tr;
        }
        sec_to_regs(cal_sec(s), &tr, &dr);
        return off ? dr : tr;
    case 0x08: return s->cr;
    case 0x0C:
        return s->isr | ISR_ALRAWF | ISR_ALRBWF | ISR_WUTWF | ISR_RSF |
               (s->init_mode ? ISR_INITF | ISR_INIT : 0);
    case 0x10: return s->prer;
    case 0x14: return s->wutr;
    case 0x1C: return s->alrmar;
    case 0x20: return s->alrmbr;
    case 0x28: return ssr(s);
    case 0x3C: return s->calr;
    case 0x40: return s->tampcr;
    case 0x44: return s->alrmassr;
    case 0x48: return s->alrmbssr;
    case 0x4C: return s->or;
    }
    return 0;
}

static void rtc_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7RtcState *s = opaque;

    if (off >= 0x50 && off < 0x50 + 4 * NUM_BKP) {
        s->bkp[(off - 0x50) / 4] = v;
        return;
    }
    switch (off) {
    case 0x00: if (s->init_mode) { s->tr = v & 0x007F7F7F; } break;
    case 0x04: if (s->init_mode) { s->dr = v & 0x00FFFF3F; } break;
    case 0x08: {
        uint32_t old = s->cr;
        s->cr = v;
        if ((v & CR_WUTE) && !(old & CR_WUTE)) {
            timer_mod(s->wut, now_ns() + wut_period_ns(s));
        } else if (!(v & CR_WUTE)) {
            timer_del(s->wut);
        }
        break;
    }
    case 0x0C:
        if ((v & ISR_INIT) && !s->init_mode) {
            sec_to_regs(cal_sec(s), &s->tr, &s->dr);
            s->init_mode = true;
        } else if (!(v & ISR_INIT) && s->init_mode) {
            s->init_mode = false;
            s->base_sec = regs_to_sec(s->tr, s->dr);
            s->base_ns = now_ns();
            s->isr |= ISR_INITS;
            tick_arm(s);
        }
        /* flags are cleared by writing 0 */
        s->isr &= v | ~(ISR_ALRAF | ISR_ALRBF | ISR_WUTF);
        break;
    case 0x10: s->prer = v & 0x007F7FFF; break;
    case 0x14: s->wutr = v & 0xFFFF; break;
    case 0x1C: s->alrmar = v; break;
    case 0x20: s->alrmbr = v; break;
    case 0x24: break;       /* write protection key: not enforced */
    case 0x3C: s->calr = v; break;
    case 0x40: s->tampcr = v; break;
    case 0x44: s->alrmassr = v; break;
    case 0x48: s->alrmbssr = v; break;
    case 0x4C: s->or = v; break;
    default:
        qemu_log_mask(LOG_UNIMP, "stm32h7-rtc: write 0x%" HWADDR_PRIx "\n", off);
    }
    update_irq(s);
}

static const MemoryRegionOps rtc_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Backup domain: calendar and backup registers keep their values */
static void rtc_reset(DeviceState *dev)
{
    Stm32h7RtcState *s = STM32H7_RTC(dev);

    s->cr &= 7;
    s->isr &= ISR_INITS;
    s->init_mode = false;
    timer_del(s->wut);
    update_irq(s);
}

static void rtc_realize(DeviceState *dev, Error **errp)
{
    Stm32h7RtcState *s = STM32H7_RTC(dev);
    struct tm tm;

    memory_region_init_io(&s->iomem, OBJECT(s), &rtc_ops, s, "stm32h7-rtc", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->alarm_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->wkup_irq);
    s->tick = timer_new_ns(rtc_clock, tick_cb, s);
    s->wut = timer_new_ns(rtc_clock, wut_cb, s);

    qemu_get_timedate(&tm, 0);
    s->base_sec = mktimegm(&tm);
    s->base_ns = now_ns();
    s->prer = 0x007F00FF;
    s->isr = ISR_INITS;     /* the calendar already holds a date */
    tick_arm(s);
}

static void rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = rtc_realize;
    device_class_set_legacy_reset(dc, rtc_reset);
}

static const TypeInfo rtc_info = {
    .name = TYPE_STM32H7_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7RtcState),
    .class_init = rtc_class_init,
};

static void rtc_register(void)
{
    type_register_static(&rtc_info);
}
type_init(rtc_register)
