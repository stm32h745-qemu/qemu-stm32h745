/*
 * STM32G4 ADC (RM0440 chapter 21): a bank of ADC instances sharing a common
 * register block (ADC1+ADC2, or ADC3).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ADC_STM32G4_ADC_H
#define HW_ADC_STM32G4_ADC_H

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_STM32G4_ADC "stm32g4-adc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4AdcState, STM32G4_ADC)

#define STM32G4_ADC_MAX_INST   3
#define STM32G4_ADC_CHANNELS   19
#define STM32G4_ADC_TRIGGERS   32

/* An analog input whose value depends on time: mV at virtual time now_ns */
typedef double (*Stm32g4AdcInputFn)(void *opaque, int64_t now_ns);

typedef struct Stm32g4AdcInst {
    uint32_t isr, ier, cr, cfgr, cfgr2, smpr[2], tr[3], sqr[4], dr, jsqr;
    uint32_t ofr[4], jdr[4], awd2cr, awd3cr, difsel, calfact, gcomp;

    bool started;           /* ADSTART */
    bool calibrating;
    bool converting;        /* a conversion is in flight */
    bool waiting;           /* sequence paused (discontinuous), waits for a trigger */
    int phase;              /* 0: waiting for the sampling instant, 1: for the end */
    int rank;               /* 0-based rank being converted */
    int discnt;
    int64_t t_sample, t_end;
    uint32_t raw;           /* value captured at the sampling instant */
    bool trig_level[STM32G4_ADC_TRIGGERS];
    QEMUTimer *timer;
    QEMUTimer *cal_timer;
    qemu_irq dma_req;
    double mv[STM32G4_ADC_CHANNELS];
    Stm32g4AdcInputFn fn[STM32G4_ADC_CHANNELS];
    void *fn_opaque[STM32G4_ADC_CHANNELS];
    struct Stm32g4AdcState *bank;
    int index;
} Stm32g4AdcInst;

struct Stm32g4AdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    Stm32g4AdcInst adc[STM32G4_ADC_MAX_INST];
    uint32_t ccr;

    uint32_t num;           /* instances in this bank */
    uint32_t first;         /* absolute number of the first (1 or 3) */
    uint32_t vref_mv;
    uint32_t common_base;   /* offset of the common block (0x300) */
    uint64_t (*kernel_hz)(void *opaque);    /* ADC kernel clock */
    uint64_t (*hclk_hz)(void *opaque);
    void *clk_opaque;
};

void stm32g4_adc_set_input_mv(Stm32g4AdcState *s, int inst, int ch, double mv);
/* Attach a time dependent input to ADC number adc_num (1-based) channel ch */
bool stm32g4_adc_attach_input(int adc_num, int ch, Stm32g4AdcInputFn fn, void *opaque,
                              Error **errp);

#endif
