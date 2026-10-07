/*
 * STM32H7 ADC block (RM0399 chapter 25): ADC1+ADC2 or ADC3, each followed by
 * the common registers at offset 0x300.
 *
 * Modeled: voltage regulator and deep power-down bits, calibration (ADCAL
 * completes at once), enable/disable (ADRDY), regular sequences (SQR1-4,
 * length L) in single or continuous mode with EOC/EOS, injected conversions
 * (JSQR, JEOC/JEOS), interrupts (IER). Each channel's analog input is a
 * voltage in mV (stm32h7_adc_set_input_mv(); 0 by default); a conversion
 * returns it scaled to the configured resolution against vref-mv. The RES
 * field is decoded for both silicon revisions (Y and V).
 *
 * Not modeled: conversion and sampling time, oversampling, offsets, analog
 * watchdogs, differential mode, DMA, the multi-ADC modes.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/adc/stm32h7_adc.h"

#define ISR_ADRDY  BIT(0)
#define ISR_EOC    BIT(2)
#define ISR_EOS    BIT(3)
#define ISR_JEOC   BIT(5)
#define ISR_JEOS   BIT(6)
#define ISR_LDORDY BIT(12)

#define CR_ADEN     BIT(0)
#define CR_ADDIS    BIT(1)
#define CR_ADSTART  BIT(2)
#define CR_JADSTART BIT(3)
#define CR_ADSTP    BIT(4)
#define CR_JADSTP   BIT(5)
#define CR_ADVREGEN BIT(28)
#define CR_DEEPPWD  BIT(29)
#define CR_ADCAL    BIT(31)

#define CFGR_CONT   BIT(13)

#define COMMON_OFF  0x300

static unsigned res_bits(Stm32h7AdcUnit *u)
{
    /* RES[4:2]; revision Y: 0-4 = 16/14/12/10/8 bits, revision V: 0/5/6/3/7 */
    static const unsigned bits[8] = { 16, 14, 12, 10, 8, 14, 12, 8 };
    return bits[(u->cfgr >> 2) & 7];
}

static uint32_t convert(Stm32h7AdcState *s, Stm32h7AdcUnit *u, unsigned ch)
{
    uint64_t full = (1ULL << res_bits(u)) - 1;
    uint64_t mv = ch < STM32H7_ADC_CHANNELS ? u->in_mv[ch] : 0;
    uint64_t v = (mv * full + s->vref_mv / 2) / s->vref_mv;

    return MIN(v, full);
}

/* Channel of regular rank r (0-based) */
static unsigned rank_channel(Stm32h7AdcUnit *u, unsigned r)
{
    /* SQ1-4 in SQR1 [10:6]...[28:24]; then 5 per register from bit 0 */
    if (r < 4) {
        return (u->sqr[0] >> (6 + 6 * r)) & 0x1F;
    }
    r -= 4;
    return (u->sqr[1 + r / 5] >> (6 * (r % 5))) & 0x1F;
}

static unsigned seq_len(Stm32h7AdcUnit *u)
{
    return (u->sqr[0] & 0xF) + 1;
}

static void update_irq(Stm32h7AdcState *s)
{
    bool lvl = false;

    for (unsigned i = 0; i < s->num; i++) {
        lvl |= (s->adc[i].isr & s->adc[i].ier) != 0;
    }
    qemu_set_irq(s->irq, lvl);
}

/* Convert the current rank; called on start and after each DR read */
static void regular_step(Stm32h7AdcState *s, Stm32h7AdcUnit *u)
{
    u->dr = convert(s, u, rank_channel(u, u->seq_pos));
    u->isr |= ISR_EOC;
    if (++u->seq_pos >= seq_len(u)) {
        u->isr |= ISR_EOS;
        u->seq_pos = 0;
        if (!(u->cfgr & CFGR_CONT)) {
            u->cr &= ~CR_ADSTART;
        }
    }
}

static void injected_run(Stm32h7AdcState *s, Stm32h7AdcUnit *u)
{
    unsigned n = (u->jsqr & 3) + 1;

    for (unsigned i = 0; i < n; i++) {
        u->jdr[i] = convert(s, u, (u->jsqr >> (9 + 6 * i)) & 0x1F);
    }
    u->isr |= ISR_JEOC | ISR_JEOS;
    u->cr &= ~CR_JADSTART;
}

static void cr_write(Stm32h7AdcState *s, Stm32h7AdcUnit *u, uint32_t v)
{
    uint32_t keep = u->cr & (CR_ADEN | CR_ADSTART | CR_JADSTART);

    u->cr = (v & ~(CR_ADDIS | CR_ADSTP | CR_JADSTP | CR_ADCAL |
                   CR_ADEN | CR_ADSTART | CR_JADSTART)) | keep;
    if (u->cr & CR_ADVREGEN) {
        u->isr |= ISR_LDORDY;
    }
    if (v & CR_ADCAL) {
        u->calfact = 0x00400040;    /* calibration done at once */
    }
    if (v & CR_ADDIS) {
        u->cr &= ~(CR_ADEN | CR_ADSTART | CR_JADSTART);
        u->isr &= ~ISR_ADRDY;
    }
    if ((v & CR_ADEN) && !(v & CR_ADDIS)) {
        u->cr |= CR_ADEN;
        u->isr |= ISR_ADRDY;
    }
    if (v & CR_ADSTP) {
        u->cr &= ~CR_ADSTART;
    }
    if (v & CR_JADSTP) {
        u->cr &= ~CR_JADSTART;
    }
    if ((v & CR_ADSTART) && (u->cr & CR_ADEN)) {
        u->cr |= CR_ADSTART;
        u->seq_pos = 0;
        regular_step(s, u);
    }
    if ((v & CR_JADSTART) && (u->cr & CR_ADEN)) {
        u->cr |= CR_JADSTART;
        injected_run(s, u);
    }
}

static uint32_t *unit_reg(Stm32h7AdcUnit *u, hwaddr off)
{
    switch (off) {
    case 0x04: return &u->ier;
    case 0x0C: return &u->cfgr;
    case 0x10: return &u->cfgr2;
    case 0x14: return &u->smpr1;
    case 0x18: return &u->smpr2;
    case 0x1C: return &u->pcsel;
    case 0x20: return &u->ltr1;
    case 0x24: return &u->htr1;
    case 0x30: case 0x34: case 0x38: case 0x3C: return &u->sqr[(off - 0x30) / 4];
    case 0x4C: return &u->jsqr;
    case 0x60: case 0x64: case 0x68: case 0x6C: return &u->ofr[(off - 0x60) / 4];
    case 0xC0: return &u->difsel;
    case 0xC4: return &u->calfact;
    case 0xC8: return &u->calfact2;
    }
    return NULL;
}

static uint64_t adc_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7AdcState *s = opaque;
    uint64_t v = 0;

    if (off >= COMMON_OFF) {
        switch (off - COMMON_OFF) {
        case 0x00:      /* CSR mirrors the ADC flags */
            for (unsigned i = 0; i < s->num; i++) {
                v |= (uint64_t)(s->adc[i].isr & 0x7FF) << (16 * i);
            }
            break;
        case 0x08: v = s->ccr; break;
        case 0x0C: v = s->adc[0].dr | ((uint64_t)s->adc[1].dr << 16); break;
        }
        return v;
    }
    if (off / 0x100 >= s->num) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-adc: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
    Stm32h7AdcUnit *u = &s->adc[off / 0x100];
    off &= 0xFF;
    switch (off) {
    case 0x00: v = u->isr; break;
    case 0x08: v = u->cr; break;
    case 0x40:
        v = u->dr;
        u->isr &= ~ISR_EOC;
        if (u->cr & CR_ADSTART) {
            regular_step(s, u);
        }
        break;
    case 0x80: case 0x84: case 0x88: case 0x8C: v = u->jdr[(off - 0x80) / 4]; break;
    default: {
        uint32_t *r = unit_reg(u, off);
        v = r ? *r : 0;
    }
    }
    update_irq(s);
    return v;
}

static void adc_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7AdcState *s = opaque;

    if (off >= COMMON_OFF) {
        if (off - COMMON_OFF == 0x08) {
            s->ccr = v;
        }
        return;
    }
    if (off / 0x100 >= s->num) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-adc: write 0x%" HWADDR_PRIx "\n", off);
        return;
    }
    Stm32h7AdcUnit *u = &s->adc[off / 0x100];
    off &= 0xFF;
    switch (off) {
    case 0x00: u->isr &= ~v; break;     /* write 1 to clear */
    case 0x08: cr_write(s, u, v); break;
    default: {
        uint32_t *r = unit_reg(u, off);
        if (r) {
            *r = v;
        }
    }
    }
    update_irq(s);
}

static const MemoryRegionOps adc_ops = {
    .read = adc_read,
    .write = adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

void stm32h7_adc_set_input_mv(Stm32h7AdcState *s, int unit, int ch, uint32_t mv)
{
    if (unit >= 0 && unit < STM32H7_ADC_MAX && ch >= 0 && ch < STM32H7_ADC_CHANNELS) {
        s->adc[unit].in_mv[ch] = mv;
    }
}

static void adc_reset(DeviceState *dev)
{
    Stm32h7AdcState *s = STM32H7_ADC(dev);

    for (unsigned i = 0; i < STM32H7_ADC_MAX; i++) {
        Stm32h7AdcUnit *u = &s->adc[i];
        uint32_t in[STM32H7_ADC_CHANNELS];

        memcpy(in, u->in_mv, sizeof(in));
        memset(u, 0, sizeof(*u));
        memcpy(u->in_mv, in, sizeof(in));
        u->cr = CR_DEEPPWD;
    }
    s->csr = s->ccr = 0;
}

static void adc_realize(DeviceState *dev, Error **errp)
{
    Stm32h7AdcState *s = STM32H7_ADC(dev);

    if (s->num < 1 || s->num > STM32H7_ADC_MAX || !s->vref_mv) {
        error_setg(errp, "stm32h7-adc: num must be 1 or 2, vref-mv nonzero");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &adc_ops, s, "stm32h7-adc", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static const Property adc_props[] = {
    DEFINE_PROP_UINT32("num", Stm32h7AdcState, num, 1),
    DEFINE_PROP_UINT32("vref-mv", Stm32h7AdcState, vref_mv, 3300),
};

static void adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = adc_realize;
    device_class_set_legacy_reset(dc, adc_reset);
    device_class_set_props(dc, adc_props);
}

static const TypeInfo adc_info = {
    .name = TYPE_STM32H7_ADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7AdcState),
    .class_init = adc_class_init,
};

static void adc_register(void)
{
    type_register_static(&adc_info);
}
type_init(adc_register)
