/*
 * Generic AC source with an optocoupler-style output.
 *
 * A sine wave v(t) = offset + amplitude * sin(2 pi f t + phase), of
 * configurable frequency (milli-Hz resolution), evaluated on QEMU's virtual
 * clock. Outputs:
 *
 *  - digital: the "optocoupler" conducts while |v| >= threshold (so it drops
 *    out in a window around every zero crossing); "half-wave" restricts it to
 *    the positive (v >= threshold) or negative (v <= -threshold) half wave.
 *    The output level (conducting = high, or low with "invert") is driven
 *    onto a GPIO input pin ("gpio", e.g. "C5"); edges are scheduled on the
 *    virtual clock at the exact crossing times.
 *
 *  - analog: into one or more ADC channels ("adc", e.g. "2.8" or "1.12,2.12";
 *    ADC number, channel), evaluated at the instant the ADC samples. Mode
 *    "sine": the waveform in mV, after an optional first-order RC low pass
 *    ("rc-us", steady state gain and phase lag). Mode "opto": the node an
 *    optocoupler transistor with a pull-up drives: "opto-high-mv" while it
 *    does not conduct, "opto-low-mv" while it does, with first-order RC edges.
 *    The node charges with "rc-us" and discharges (conducting) with
 *    "rc-fall-us" when that is set: a filter capacitor on the node gives a
 *    ripple between two levels instead of a full swing while AC is present.
 *
 * Everything can be changed at runtime with qom-set (frequency, phase,
 * amplitude/offset/threshold, enable): the phase stays continuous across a
 * frequency change, a phase change steps the wave. Any number of instances.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/stm32g4_gpio.h"
#include "hw/adc/stm32g4_adc.h"
#include "qom/object.h"
#include "trace.h"
#include <math.h>

#define TYPE_AC_SOURCE "ac-source"
OBJECT_DECLARE_SIMPLE_TYPE(AcSourceState, AC_SOURCE)

#define MAX_ADC_TARGETS 8

struct AcSourceState {
    DeviceState parent_obj;

    /* waveform */
    uint64_t freq_mhz;          /* milli-Hz */
    double phase_off;           /* turns, added to the phase */
    double amplitude, offset;   /* mV */
    double threshold;           /* mV */
    int half_wave;              /* 0 full wave, 1 positive, 2 negative */
    bool enabled;
    bool invert;
    double rc_us;
    double rc_fall_us;          /* < 0: same as rc_us */
    double opto_high, opto_low;
    char *analog_mode;          /* "sine" or "opto" */
    bool opto_analog;

    /* time base: phase (turns, without phase_off) was base_turns at base_t */
    int64_t base_t;
    double base_turns;

    /* optocoupler state and the analog edge reference */
    bool conducting;
    int64_t edge_t;
    double edge_v;

    char *gpio_name;
    char *adc_list;
    Stm32g4GpioState *gpio;
    int pin;
    QEMUTimer *timer;
};

static double turns_at(AcSourceState *s, int64_t t)
{
    return s->base_turns + (double)(t - s->base_t) * 1e-9 * ((double)s->freq_mhz * 1e-3) +
           s->phase_off;
}

static double wave_mv(AcSourceState *s, int64_t t)
{
    double th = turns_at(s, t);

    return s->offset + s->amplitude * sin(2.0 * M_PI * (th - floor(th)));
}

static bool conducting_at(AcSourceState *s, int64_t t)
{
    double v;

    if (!s->enabled) {
        return false;
    }
    v = wave_mv(s, t);
    switch (s->half_wave) {
    case 1: return v >= s->threshold;
    case 2: return v <= -s->threshold;
    }
    return fabs(v) >= s->threshold;
}

/* Voltage of the optocoupler node at t (levels with RC edges) */
static double opto_mv(AcSourceState *s, int64_t t)
{
    double target = s->conducting ? s->opto_low : s->opto_high;
    double tau = (s->conducting && s->rc_fall_us >= 0 ? s->rc_fall_us : s->rc_us) * 1e3; /* ns */

    if (tau <= 0 || t <= s->edge_t) {
        return tau <= 0 ? target : s->edge_v;
    }
    return target + (s->edge_v - target) * exp(-(double)(t - s->edge_t) / tau);
}

static double analog_fn(void *opaque, int64_t t)
{
    AcSourceState *s = opaque;

    if (s->opto_analog) {
        return opto_mv(s, t);
    }
    if (!s->enabled) {
        return 0;
    }
    if (s->rc_us > 0) {
        double w = 2.0 * M_PI * ((double)s->freq_mhz * 1e-3);
        double wt = w * s->rc_us * 1e-6;
        double g = 1.0 / sqrt(1.0 + wt * wt);
        double th = turns_at(s, t) - atan(wt) / (2.0 * M_PI);

        return s->offset + s->amplitude * g * sin(2.0 * M_PI * (th - floor(th)));
    }
    return wave_mv(s, t);
}

static void drive_gpio(AcSourceState *s)
{
    if (s->gpio) {
        stm32g4_gpio_set_input(s->gpio, s->pin, s->conducting ^ s->invert);
    }
}

/* Delay in ns from t to the next time the conduction state can change, or -1 */
static int64_t next_edge_delay(AcSourceState *s, int64_t t)
{
    double f = (double)s->freq_mhz * 1e-3;
    double th = turns_at(s, t);
    double best = 2.0;
    double levels[2];
    int nlev = 0;

    if (!s->enabled || s->amplitude <= 0 || f <= 0) {
        return -1;
    }
    if (s->half_wave != 2) {
        levels[nlev++] = s->threshold;
    }
    if (s->half_wave != 1) {
        levels[nlev++] = -s->threshold;
    }
    for (int i = 0; i < nlev; i++) {
        double x = (levels[i] - s->offset) / s->amplitude;
        double p1, p2;

        if (x > 1.0 || x < -1.0) {
            continue;
        }
        p1 = asin(x) / (2.0 * M_PI);
        p2 = 0.5 - p1;
        for (int k = 0; k < 2; k++) {
            double d = (k ? p2 : p1) - th;

            d -= floor(d);                  /* in [0, 1) */
            if (d < 1e-12) {
                d = 1.0;
            }
            best = MIN(best, d);
        }
    }
    if (best > 1.5) {
        return -1;
    }
    return (int64_t)ceil(best / f * 1e9);
}

static void reschedule(AcSourceState *s, int64_t now)
{
    int64_t d = next_edge_delay(s, now);

    if (d < 0) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, now + MAX(d, 1));
    }
}

/* Apply a new conduction state at time t */
static void set_state(AcSourceState *s, bool state, int64_t t)
{
    if (state == s->conducting) {
        return;
    }
    s->edge_v = opto_mv(s, t);
    s->edge_t = t;
    s->conducting = state;
    trace_ac_source_edge(t, state);
    drive_gpio(s);
}

static void edge_cb(void *opaque)
{
    AcSourceState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* evaluate just after the crossing */
    set_state(s, conducting_at(s, now + 1), now);
    reschedule(s, now + 1);
}

/* A parameter changed: re-evaluate everything at the current time */
static void refresh(AcSourceState *s)
{
    int64_t now;

    if (!s->timer) {
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    set_state(s, conducting_at(s, now), now);
    reschedule(s, now);
}

/* Change the frequency keeping the phase continuous */
static void rebase(AcSourceState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->base_turns = turns_at(s, now) - s->phase_off;
    s->base_turns -= floor(s->base_turns);
    s->base_t = now;
}

enum {
    P_FREQ, P_PHASE, P_AMP, P_OFFSET, P_THR, P_ENABLED, P_HALF, P_INVERT, P_RC, P_OHI, P_OLO, P_RCF,
};

static void prop_get(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    AcSourceState *s = AC_SOURCE(obj);
    double d;
    uint64_t u;
    bool b;
    int64_t i;

    switch ((intptr_t)opaque) {
    case P_FREQ: u = s->freq_mhz; visit_type_uint64(v, name, &u, errp); return;
    case P_PHASE: d = s->phase_off * 360.0; break;
    case P_AMP: d = s->amplitude; break;
    case P_OFFSET: d = s->offset; break;
    case P_THR: d = s->threshold; break;
    case P_ENABLED: b = s->enabled; visit_type_bool(v, name, &b, errp); return;
    case P_HALF: i = s->half_wave; visit_type_int(v, name, &i, errp); return;
    case P_INVERT: b = s->invert; visit_type_bool(v, name, &b, errp); return;
    case P_RC: d = s->rc_us; break;
    case P_OHI: d = s->opto_high; break;
    case P_OLO: d = s->opto_low; break;
    case P_RCF: d = s->rc_fall_us; break;
    default: return;
    }
    visit_type_number(v, name, &d, errp);
}

static void prop_set(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    AcSourceState *s = AC_SOURCE(obj);
    double d = 0;
    uint64_t u;
    bool b;
    int64_t i;

    switch ((intptr_t)opaque) {
    case P_FREQ:
        if (!visit_type_uint64(v, name, &u, errp)) {
            return;
        }
        rebase(s);
        s->freq_mhz = u;
        break;
    case P_ENABLED:
        if (!visit_type_bool(v, name, &b, errp)) {
            return;
        }
        s->enabled = b;
        break;
    case P_INVERT:
        if (!visit_type_bool(v, name, &b, errp)) {
            return;
        }
        s->invert = b;
        drive_gpio(s);
        return;
    case P_HALF:
        if (!visit_type_int(v, name, &i, errp)) {
            return;
        }
        if (i < 0 || i > 2) {
            error_setg(errp, "half-wave: 0 full wave, 1 positive, 2 negative");
            return;
        }
        s->half_wave = i;
        break;
    default:
        if (!visit_type_number(v, name, &d, errp)) {
            return;
        }
        switch ((intptr_t)opaque) {
        case P_PHASE: s->phase_off = d / 360.0; break;
        case P_AMP: s->amplitude = d; break;
        case P_OFFSET: s->offset = d; break;
        case P_THR: s->threshold = d; break;
        case P_RC: s->rc_us = d; return;
        case P_OHI: s->opto_high = d; return;
        case P_OLO: s->opto_low = d; return;
        case P_RCF: s->rc_fall_us = d; return;
        }
    }
    refresh(s);
}

static void add_prop(Object *obj, const char *name, const char *type, intptr_t id,
                     const char *desc)
{
    object_property_add(obj, name, type, prop_get, prop_set, NULL, (void *)id);
    object_property_set_description(obj, name, desc);
}

static void ac_init(Object *obj)
{
    AcSourceState *s = AC_SOURCE(obj);

    s->freq_mhz = 60000;
    s->amplitude = 1.0;
    s->threshold = 0.1;
    s->enabled = true;
    s->opto_high = 3300;
    s->opto_low = 0;
    s->rc_fall_us = -1;
    add_prop(obj, "freq-mhz", "uint64", P_FREQ, "frequency in milli-Hz");
    add_prop(obj, "phase-deg", "number", P_PHASE, "phase at t=0 in degrees (runtime change steps the wave)");
    add_prop(obj, "amplitude-mv", "number", P_AMP, "sine amplitude (mV)");
    add_prop(obj, "offset-mv", "number", P_OFFSET, "DC offset (mV)");
    add_prop(obj, "threshold-mv", "number", P_THR, "optocoupler conduction threshold (mV)");
    add_prop(obj, "enabled", "bool", P_ENABLED, "source on/off");
    add_prop(obj, "half-wave", "int", P_HALF, "0 full wave, 1 positive only, 2 negative only");
    add_prop(obj, "invert", "bool", P_INVERT, "digital output low while conducting");
    add_prop(obj, "rc-us", "number", P_RC, "first order RC time constant of the analog output (us)");
    add_prop(obj, "rc-fall-us", "number", P_RCF,
             "opto mode: RC time constant while conducting (us); < 0 uses rc-us");
    add_prop(obj, "opto-high-mv", "number", P_OHI, "opto analog node level while not conducting (mV)");
    add_prop(obj, "opto-low-mv", "number", P_OLO, "opto analog node level while conducting (mV)");
}

static bool parse_gpio(AcSourceState *s, Error **errp)
{
    char port;
    int pin;
    g_autofree char *path = NULL;
    Object *o;

    if (!s->gpio_name || !*s->gpio_name) {
        return true;
    }
    if (sscanf(s->gpio_name, "%c%d", &port, &pin) != 2 || !g_ascii_isalpha(port) ||
        pin < 0 || pin > 15) {
        error_setg(errp, "gpio: expected a pin like 'C5', got '%s'", s->gpio_name);
        return false;
    }
    path = g_strdup_printf("/machine/soc/gpio%c", g_ascii_tolower(port));
    o = object_resolve_path_type(path, TYPE_STM32G4_GPIO, NULL);
    if (!o) {
        error_setg(errp, "gpio: no GPIO port %c in this machine", g_ascii_toupper(port));
        return false;
    }
    s->gpio = STM32G4_GPIO(o);
    s->pin = pin;
    return true;
}

static bool attach_adcs(AcSourceState *s, Error **errp)
{
    g_auto(GStrv) items = NULL;

    if (!s->adc_list || !*s->adc_list) {
        return true;
    }
    items = g_strsplit(s->adc_list, ",", -1);
    for (char **it = items; *it; it++) {
        unsigned adc, ch;

        if (sscanf(*it, "%u.%u", &adc, &ch) != 2) {
            error_setg(errp, "adc: expected '<adc>.<channel>[,...]', got '%s'", *it);
            return false;
        }
        if (!stm32g4_adc_attach_input(adc, ch, analog_fn, s, errp)) {
            return false;
        }
    }
    return true;
}

static void ac_realize(DeviceState *dev, Error **errp)
{
    AcSourceState *s = AC_SOURCE(dev);

    s->opto_analog = s->analog_mode && !strcmp(s->analog_mode, "opto");
    if (s->analog_mode && strcmp(s->analog_mode, "sine") && !s->opto_analog) {
        error_setg(errp, "analog-mode: 'sine' or 'opto'");
        return;
    }
    if (!parse_gpio(s, errp) || !attach_adcs(s, errp)) {
        return;
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, edge_cb, s);
    s->base_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_turns = 0;
    s->conducting = conducting_at(s, s->base_t);
    s->edge_t = s->base_t;
    s->edge_v = s->conducting ? s->opto_low : s->opto_high;
    drive_gpio(s);
    reschedule(s, s->base_t);
}

static const Property ac_props[] = {
    DEFINE_PROP_STRING("gpio", AcSourceState, gpio_name),
    DEFINE_PROP_STRING("adc", AcSourceState, adc_list),
    DEFINE_PROP_STRING("analog-mode", AcSourceState, analog_mode),
};

static void ac_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ac_realize;
    device_class_set_props(dc, ac_props);
    dc->desc = "AC source with optocoupler output (GPIO input and/or ADC input)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo ac_info = {
    .name = TYPE_AC_SOURCE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(AcSourceState),
    .instance_init = ac_init,
    .class_init = ac_class_init,
};

static void ac_register(void)
{
    type_register_static(&ac_info);
}
type_init(ac_register)
