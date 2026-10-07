/*
 * STM32H7 ADC block: one or two ADCs and their common registers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ADC_STM32H7_ADC_H
#define HW_ADC_STM32H7_ADC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32H7_ADC "stm32h7-adc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7AdcState, STM32H7_ADC)

#define STM32H7_ADC_CHANNELS 20
#define STM32H7_ADC_MAX      2

typedef struct Stm32h7AdcUnit {
    uint32_t isr, ier, cr, cfgr, cfgr2, smpr1, smpr2, pcsel, ltr1, htr1;
    uint32_t sqr[4], dr, jsqr, ofr[4], jdr[4], difsel, calfact, calfact2;
    uint32_t seq_pos;
    uint32_t in_mv[STM32H7_ADC_CHANNELS];   /* analog input of each channel */
} Stm32h7AdcUnit;

struct Stm32h7AdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t num;                   /* ADCs in this block: 1 or 2 */
    uint32_t vref_mv;
    Stm32h7AdcUnit adc[STM32H7_ADC_MAX];
    uint32_t csr, ccr;
};

/* Set the analog input of channel ch of ADC unit (0-based) in the block */
void stm32h7_adc_set_input_mv(Stm32h7AdcState *s, int unit, int ch, uint32_t mv);

#endif
