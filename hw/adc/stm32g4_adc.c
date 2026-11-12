/*
 * STM32G4 ADC.
 *
 * Calibration, voltage regulator / deep power-down handshakes, enable/disable
 * with ADRDY, regular sequence (SQR, scan, discontinuous, continuous), per
 * channel sampling times, software and external triggers (EXTSEL/EXTEN, the
 * timers' TRGO arrive on the "trig" inputs), 12/10/8/6 bit resolution and
 * alignment, EOSMP/EOC/EOS/OVR flags and interrupts, and the DMA request.
 *
 * Timing is that of the part: the ADC clock follows the RCC settings
 * (asynchronous kernel clock / PRESC, or HCLK / CKMODE), each conversion
 * takes the channel's sampling time plus 12.5 (10.5, 8.5, 6.5) ADC cycles,
 * and the analog input is captured at the end of the sampling phase, i.e.
 * evaluated at that instant of QEMU's virtual clock. An input is either a
 * fixed voltage (mV, settable at runtime) or a function of time provided by
 * another device (the AC source).
 *
 * Not modeled: injected conversions, analog watchdogs, oversampling,
 * differential mode, gain/offset calibration and dual mode (accepted and
 * ignored, logged with -d unimp).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/adc/stm32g4_adc.h"
#include "migration/vmstate.h"

#define ISR  0x00
#define IER  0x04
#define CR   0x08
#define CFGR 0x0C
#define CFGR2 0x10
#define SMPR1 0x14
#define SMPR2 0x18
#define TR1  0x20
#define TR2  0x24
#define TR3  0x28
#define SQR1 0x30
#define SQR2 0x34
#define SQR3 0x38
#define SQR4 0x3C
#define DR   0x40
#define JSQR 0x4C
#define OFR1 0x60
#define JDR1 0x80
#define AWD2CR 0xA0
#define AWD3CR 0xA4
#define DIFSEL 0xB0
#define CALFACT 0xB4
#define GCOMP 0xC0

#define ISR_ADRDY BIT(0)
#define ISR_EOSMP BIT(1)
#define ISR_EOC   BIT(2)
#define ISR_EOS   BIT(3)
#define ISR_OVR   BIT(4)
#define ISR_W1C   0x7FF

#define CR_ADEN     BIT(0)
#define CR_ADDIS    BIT(1)
#define CR_ADSTART  BIT(2)
#define CR_JADSTART BIT(3)
#define CR_ADSTP    BIT(4)
#define CR_JADSTP   BIT(5)
#define CR_ADVREGEN BIT(28)
#define CR_DEEPPWD  BIT(29)
#define CR_ADCALDIF BIT(30)
#define CR_ADCAL    BIT(31)

#define CFGR_DMAEN  BIT(0)
#define CFGR_DMACFG BIT(1)
#define CFGR_OVRMOD BIT(12)
#define CFGR_CONT   BIT(13)
#define CFGR_ALIGN  BIT(15)
#define CFGR_DISCEN BIT(16)

/* sampling times in half ADC clock cycles: 2.5 6.5 12.5 24.5 47.5 92.5 247.5 640.5 */
static const unsigned smp_half_cycles[8] = { 5, 13, 25, 49, 95, 185, 495, 1281 };
static const unsigned presc_div[12] = { 1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256 };

#define TRIGGER_LATENCY_HC 5        /* 2.5 ADC cycles */
#define CAL_HC             (2 * 82)

static uint32_t resolution(Stm32g4AdcInst *a)
{
    return 12 - 2 * ((a->cfgr >> 3) & 3);
}

static uint32_t sar_half_cycles(Stm32g4AdcInst *a)
{
    return 2 * resolution(a) + 1;       /* 12.5, 10.5, 8.5, 6.5 cycles */
}

/* One half ADC clock period in ns, as a rational: returns ns for n half cycles */
static int64_t hc_ns(Stm32g4AdcState *b, uint64_t n)
{
    uint64_t hz = 0;
    uint32_t ckmode = (b->ccr >> 16) & 3;

    if (ckmode == 0) {
        uint32_t p = (b->ccr >> 18) & 0xF;

        hz = b->kernel_hz ? b->kernel_hz(b->clk_opaque) : 0;
        if (!hz && b->hclk_hz) {
            hz = b->hclk_hz(b->clk_opaque);
        }
        hz /= presc_div[MIN(p, 11)];
    } else {
        hz = b->hclk_hz ? b->hclk_hz(b->clk_opaque) : 0;
        hz /= 1u << (ckmode - 1);
    }
    if (!hz) {
        hz = 1000000;
    }
    /* n half cycles = n / (2 hz) seconds */
    return (int64_t)(((unsigned __int128)n * NANOSECONDS_PER_SECOND + 2 * hz - 1) /
                     (2 * (unsigned __int128)hz));
}

static void update_irq(Stm32g4AdcState *b)
{
    bool l = false;

    for (uint32_t i = 0; i < b->num; i++) {
        l |= !!(b->adc[i].isr & b->adc[i].ier);
    }
    qemu_set_irq(b->irq, l);
}

static void update_dma(Stm32g4AdcInst *a)
{
    qemu_set_irq(a->dma_req, (a->cfgr & CFGR_DMAEN) && (a->isr & ISR_EOC));
}

static unsigned sq_channel(Stm32g4AdcInst *a, unsigned rank)     /* rank is 1-based */
{
    if (rank <= 4) {
        return (a->sqr[0] >> (6 * rank)) & 0x1F;
    } else if (rank <= 9) {
        return (a->sqr[1] >> (6 * (rank - 5))) & 0x1F;
    } else if (rank <= 14) {
        return (a->sqr[2] >> (6 * (rank - 10))) & 0x1F;
    }
    return (a->sqr[3] >> (6 * (rank - 15))) & 0x1F;
}

static unsigned smp_of(Stm32g4AdcInst *a, unsigned ch)
{
    if (ch <= 9) {
        return (a->smpr[0] >> (3 * ch)) & 7;
    }
    return (a->smpr[1] >> (3 * (ch - 10))) & 7;
}

static double input_mv(Stm32g4AdcInst *a, unsigned ch, int64_t t)
{
    if (ch >= STM32G4_ADC_CHANNELS) {
        return 0;
    }
    if (a->fn[ch]) {
        return a->fn[ch](a->fn_opaque[ch], t);
    }
    return a->mv[ch];
}

static void conv_begin(Stm32g4AdcInst *a, int64_t t0)
{
    Stm32g4AdcState *b = a->bank;
    unsigned ch = sq_channel(a, a->rank + 1);

    a->converting = true;
    a->phase = 0;
    a->t_sample = t0 + hc_ns(b, smp_half_cycles[smp_of(a, ch)]);
    a->t_end = a->t_sample + hc_ns(b, sar_half_cycles(a));
    timer_mod(a->timer, a->t_sample);
}

static void begin_sequence(Stm32g4AdcInst *a)
{
    Stm32g4AdcState *b = a->bank;

    a->rank = 0;
    a->discnt = 0;
    a->waiting = false;
    conv_begin(a, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + hc_ns(b, TRIGGER_LATENCY_HC));
}

static void conv_finish(Stm32g4AdcInst *a)
{
    Stm32g4AdcState *b = a->bank;
    uint32_t res = resolution(a);
    uint32_t maxv = (1u << res) - 1;
    double v = a->raw;
    uint32_t cnt = v < 0 ? 0 : v > maxv ? maxv : (uint32_t)v;
    uint32_t nranks = (a->sqr[0] & 0xF) + 1;
    bool last = (uint32_t)a->rank + 1 >= nranks;

    (void)b;
    if (a->cfgr & CFGR_ALIGN) {
        cnt <<= 16 - res;
    }
    if (a->isr & ISR_EOC) {
        a->isr |= ISR_OVR;
        if (a->cfgr & CFGR_OVRMOD) {
            a->dr = cnt;
        }
    } else {
        a->dr = cnt;
    }
    a->isr |= ISR_EOC;
    if (last) {
        a->isr |= ISR_EOS;
    }
    update_irq(a->bank);
    update_dma(a);     /* a DMA channel serves the request before we go on */
}

static void conv_event(void *opaque)
{
    Stm32g4AdcInst *a = opaque;
    Stm32g4AdcState *b = a->bank;

    if (a->phase == 0) {
        unsigned ch = sq_channel(a, a->rank + 1);
        double mv = input_mv(a, ch, a->t_sample);
        uint32_t maxv = (1u << resolution(a)) - 1;
        double raw = mv * maxv / b->vref_mv + 0.5;

        a->raw = raw < 0 ? 0 : raw > maxv ? maxv : (uint32_t)raw;
        a->isr |= ISR_EOSMP;
        update_irq(b);
        a->phase = 1;
        timer_mod(a->timer, a->t_end);
        return;
    }
    {
        uint32_t nranks = (a->sqr[0] & 0xF) + 1;
        int64_t t_end = a->t_end;

        a->converting = false;
        conv_finish(a);
        a->rank++;
        if ((uint32_t)a->rank >= nranks) {
            /* end of the sequence */
            a->rank = 0;
            if (a->cfgr & CFGR_CONT) {
                conv_begin(a, t_end);
            } else if (!(a->cfgr & (3u << 10))) {
                a->started = false;         /* software start: single sequence */
            } else {
                a->waiting = true;          /* armed for the next trigger */
            }
            a->discnt = 0;
        } else if ((a->cfgr & CFGR_DISCEN) &&
                   ++a->discnt > (int)((a->cfgr >> 17) & 7)) {
            a->discnt = 0;
            a->waiting = true;
        } else {
            conv_begin(a, t_end);
        }
        update_irq(b);
    }
}

static void trigger(Stm32g4AdcInst *a)
{
    if (!a->started || a->converting) {
        return;
    }
    if (a->waiting || a->rank == 0) {
        a->waiting = false;
        if (a->rank == 0) {
            begin_sequence(a);
        } else {
            conv_begin(a, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          hc_ns(a->bank, TRIGGER_LATENCY_HC));
        }
    }
}

static void trig_in(void *opaque, int n, int level)
{
    Stm32g4AdcState *b = opaque;

    /* lines are numbered inst * 32 + EXTSEL; EXTSEL is also what this input
     * is connected as on every instance */
    for (uint32_t i = 0; i < b->num; i++) {
        Stm32g4AdcInst *a = &b->adc[i];
        uint32_t exten = (a->cfgr >> 10) & 3;
        bool old = a->trig_level[n];

        a->trig_level[n] = level;
        if (exten && ((a->cfgr >> 5) & 0x1F) == (uint32_t)n && old != !!level) {
            bool rising = level;

            if ((rising && (exten & 1)) || (!rising && (exten & 2))) {
                trigger(a);
            }
        }
    }
}

static void cal_done(void *opaque)
{
    Stm32g4AdcInst *a = opaque;

    a->cr &= ~CR_ADCAL;
    a->calibrating = false;
    a->calfact = 0x00130013;        /* plausible factors; value is not used */
}

static void stop_conversions(Stm32g4AdcInst *a)
{
    timer_del(a->timer);
    a->converting = false;
    a->waiting = false;
    a->started = false;
    a->rank = 0;
    a->discnt = 0;
}

static uint32_t read_cr(Stm32g4AdcInst *a)
{
    uint32_t v = a->cr & ~(CR_ADSTART | CR_ADSTP | CR_ADCAL);

    v |= a->started ? CR_ADSTART : 0;
    v |= a->calibrating ? CR_ADCAL : 0;
    return v;
}

static uint64_t adc_inst_read(Stm32g4AdcInst *a, hwaddr off)
{
    switch (off) {
    case ISR:    return a->isr;
    case IER:    return a->ier;
    case CR:     return read_cr(a);
    case CFGR:   return a->cfgr;
    case CFGR2:  return a->cfgr2;
    case SMPR1:  return a->smpr[0];
    case SMPR2:  return a->smpr[1];
    case TR1: case TR2: case TR3: return a->tr[(off - TR1) / 4];
    case SQR1: case SQR2: case SQR3: case SQR4: return a->sqr[(off - SQR1) / 4];
    case DR: {
        uint32_t v = a->dr;

        a->isr &= ~ISR_EOC;
        update_dma(a);
        update_irq(a->bank);
        return v;
    }
    case JSQR:   return a->jsqr;
    case AWD2CR: return a->awd2cr;
    case AWD3CR: return a->awd3cr;
    case DIFSEL: return a->difsel;
    case CALFACT: return a->calfact;
    case GCOMP:  return a->gcomp;
    }
    if (off >= OFR1 && off < OFR1 + 16) {
        return a->ofr[(off - OFR1) / 4];
    }
    if (off >= JDR1 && off < JDR1 + 16) {
        return a->jdr[(off - JDR1) / 4];
    }
    return 0;
}

static void adc_inst_write_cr(Stm32g4AdcInst *a, uint32_t val)
{
    Stm32g4AdcState *b = a->bank;
    bool aden = a->cr & CR_ADEN;

    if (!aden) {
        if (val & CR_DEEPPWD) {
            a->cr |= CR_DEEPPWD;
            a->cr &= ~CR_ADVREGEN;
        } else {
            a->cr &= ~CR_DEEPPWD;
            if (val & CR_ADVREGEN) {
                a->cr |= CR_ADVREGEN;
            } else {
                a->cr &= ~CR_ADVREGEN;
            }
        }
        a->cr = (a->cr & ~CR_ADCALDIF) | (val & CR_ADCALDIF);
    }
    if ((val & CR_ADCAL) && !aden && (a->cr & CR_ADVREGEN) && !a->calibrating) {
        a->calibrating = true;
        timer_mod(a->cal_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + hc_ns(b, CAL_HC));
    }
    if ((val & CR_ADEN) && !aden && !a->calibrating && (a->cr & CR_ADVREGEN) &&
        !(val & CR_ADDIS)) {
        a->cr |= CR_ADEN;
        a->isr |= ISR_ADRDY;
        update_irq(b);
    }
    if ((val & CR_ADDIS) && (a->cr & CR_ADEN) && !a->started) {
        a->cr &= ~CR_ADEN;
        a->isr &= ~ISR_ADRDY;
    }
    if ((val & CR_ADSTART) && (a->cr & CR_ADEN) && !a->started) {
        a->started = true;
        a->rank = 0;
        a->waiting = false;
        if (!((a->cfgr >> 10) & 3)) {
            begin_sequence(a);
        } else {
            a->waiting = true;      /* armed, waits for the trigger edge */
        }
    }
    if ((val & CR_ADSTP) && a->started) {
        stop_conversions(a);
    }
    if (val & (CR_JADSTART | CR_JADSTP)) {
        qemu_log_mask(LOG_UNIMP, "stm32g4-adc: injected conversions not modeled\n");
    }
}

static void adc_inst_write(Stm32g4AdcInst *a, hwaddr off, uint32_t val)
{
    switch (off) {
    case ISR:
        a->isr &= ~(val & ISR_W1C);
        update_dma(a);
        update_irq(a->bank);
        return;
    case IER:   a->ier = val & ISR_W1C; update_irq(a->bank); return;
    case CR:    adc_inst_write_cr(a, val); return;
    case CFGR:
        a->cfgr = val;
        update_dma(a);
        return;
    case CFGR2:
        a->cfgr2 = val;
        if (val & 3) {
            qemu_log_mask(LOG_UNIMP, "stm32g4-adc: oversampling not modeled\n");
        }
        return;
    case SMPR1: a->smpr[0] = val; return;
    case SMPR2: a->smpr[1] = val; return;
    case TR1: case TR2: case TR3: a->tr[(off - TR1) / 4] = val; return;
    case SQR1: case SQR2: case SQR3: case SQR4: a->sqr[(off - SQR1) / 4] = val; return;
    case JSQR:  a->jsqr = val; return;
    case AWD2CR: a->awd2cr = val; return;
    case AWD3CR: a->awd3cr = val; return;
    case DIFSEL: a->difsel = val; return;
    case CALFACT: a->calfact = val; return;
    case GCOMP: a->gcomp = val; return;
    }
    if (off >= OFR1 && off < OFR1 + 16) {
        a->ofr[(off - OFR1) / 4] = val;
    }
}

static uint64_t adc_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4AdcState *b = opaque;

    if (off >= b->common_base) {
        switch (off - b->common_base) {
        case 0x00: {
            uint32_t v = b->adc[0].isr & 0x7FF;

            if (b->num > 1) {
                v |= (b->adc[1].isr & 0x7FF) << 16;
            }
            return v;
        }
        case 0x08: return b->ccr;
        case 0x0C: return 0;
        }
        return 0;
    }
    if (off / 0x100 < b->num) {
        return adc_inst_read(&b->adc[off / 0x100], off % 0x100);
    }
    return 0;
}

static void adc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4AdcState *b = opaque;

    if (off >= b->common_base) {
        if (off - b->common_base == 0x08) {
            b->ccr = val;
            if (val & 0x1F) {
                qemu_log_mask(LOG_UNIMP, "stm32g4-adc: dual mode not modeled\n");
            }
        }
        return;
    }
    if (off / 0x100 < b->num) {
        adc_inst_write(&b->adc[off / 0x100], off % 0x100, val);
    }
}

static const MemoryRegionOps adc_ops = {
    .read = adc_read,
    .write = adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void adc_reset(DeviceState *dev)
{
    Stm32g4AdcState *b = STM32G4_ADC(dev);

    b->ccr = 0;
    for (uint32_t i = 0; i < b->num; i++) {
        Stm32g4AdcInst *a = &b->adc[i];

        timer_del(a->timer);
        timer_del(a->cal_timer);
        a->isr = a->ier = a->cfgr2 = a->dr = a->jsqr = 0;
        a->smpr[0] = a->smpr[1] = 0;
        memset(a->sqr, 0, sizeof(a->sqr));
        memset(a->ofr, 0, sizeof(a->ofr));
        memset(a->jdr, 0, sizeof(a->jdr));
        a->awd2cr = a->awd3cr = a->difsel = a->calfact = a->gcomp = 0;
        a->tr[0] = a->tr[1] = a->tr[2] = 0x0FFF0000;
        a->cr = CR_DEEPPWD;
        a->cfgr = 0x80000000;
        a->started = a->calibrating = a->converting = a->waiting = false;
        a->rank = a->discnt = 0;
        memset(a->trig_level, 0, sizeof(a->trig_level));
        update_dma(a);
    }
    update_irq(b);
}

static void in_get(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    Stm32g4AdcState *b = STM32G4_ADC(obj);
    int idx = (intptr_t)opaque;
    double mv = b->adc[idx / STM32G4_ADC_CHANNELS].mv[idx % STM32G4_ADC_CHANNELS];

    visit_type_number(v, name, &mv, errp);
}

static void in_set(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    Stm32g4AdcState *b = STM32G4_ADC(obj);
    int idx = (intptr_t)opaque;
    double mv;

    if (!visit_type_number(v, name, &mv, errp)) {
        return;
    }
    b->adc[idx / STM32G4_ADC_CHANNELS].mv[idx % STM32G4_ADC_CHANNELS] = mv;
}

void stm32g4_adc_set_input_mv(Stm32g4AdcState *s, int inst, int ch, double mv)
{
    if (inst >= 0 && inst < STM32G4_ADC_MAX_INST && ch >= 0 && ch < STM32G4_ADC_CHANNELS) {
        s->adc[inst].mv[ch] = mv;
    }
}

bool stm32g4_adc_attach_input(int adc_num, int ch, Stm32g4AdcInputFn fn, void *opaque,
                              Error **errp)
{
    const char *path = adc_num <= 2 ? "/machine/soc/adc12" : "/machine/soc/adc3";
    Object *o = object_resolve_path_type(path, TYPE_STM32G4_ADC, NULL);
    Stm32g4AdcState *b;
    int inst;

    if (!o) {
        error_setg(errp, "no ADC bank at %s", path);
        return false;
    }
    b = STM32G4_ADC(o);
    inst = adc_num - b->first;
    if (inst < 0 || inst >= (int)b->num || ch < 0 || ch >= STM32G4_ADC_CHANNELS) {
        error_setg(errp, "no ADC%d channel %d", adc_num, ch);
        return false;
    }
    b->adc[inst].fn[ch] = fn;
    b->adc[inst].fn_opaque[ch] = opaque;
    return true;
}

static void adc_realize(DeviceState *dev, Error **errp)
{
    Stm32g4AdcState *b = STM32G4_ADC(dev);

    if (b->num < 1 || b->num > STM32G4_ADC_MAX_INST) {
        error_setg(errp, "stm32g4-adc: num must be 1..%d", STM32G4_ADC_MAX_INST);
        return;
    }
    for (uint32_t i = 0; i < b->num; i++) {
        Stm32g4AdcInst *a = &b->adc[i];

        a->index = i;
        a->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, conv_event, a);
        a->cal_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cal_done, a);
        /* the analog input of each channel in mV: qom-set adcN-inM-mv */
        for (int ch = 0; ch < STM32G4_ADC_CHANNELS; ch++) {
            g_autofree char *name = g_strdup_printf("adc%u-in%d-mv", b->first + i, ch);

            object_property_add(OBJECT(dev), name, "number", in_get, in_set, NULL,
                                (void *)(intptr_t)(i * STM32G4_ADC_CHANNELS + ch));
        }
    }
}

static void adc_init(Object *obj)
{
    Stm32g4AdcState *b = STM32G4_ADC(obj);

    memory_region_init_io(&b->iomem, obj, &adc_ops, b, TYPE_STM32G4_ADC, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &b->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &b->irq);
    qdev_init_gpio_in_named(DEVICE(obj), trig_in, "trig", STM32G4_ADC_TRIGGERS);
    for (int i = 0; i < STM32G4_ADC_MAX_INST; i++) {
        g_autofree char *name = g_strdup_printf("dma-req%d", i);

        b->adc[i].bank = b;
        qdev_init_gpio_out_named(DEVICE(obj), &b->adc[i].dma_req, name, 1);
    }
}

static const Property adc_props[] = {
    DEFINE_PROP_UINT32("num", Stm32g4AdcState, num, 2),
    DEFINE_PROP_UINT32("first", Stm32g4AdcState, first, 1),
    DEFINE_PROP_UINT32("vref-mv", Stm32g4AdcState, vref_mv, 3300),
    DEFINE_PROP_UINT32("common-base", Stm32g4AdcState, common_base, 0x300),
};

static void adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = adc_realize;
    device_class_set_legacy_reset(dc, adc_reset);
    device_class_set_props(dc, adc_props);
}

static const TypeInfo adc_info = {
    .name = TYPE_STM32G4_ADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4AdcState),
    .instance_init = adc_init,
    .class_init = adc_class_init,
};

static void adc_register(void)
{
    type_register_static(&adc_info);
}
type_init(adc_register)
