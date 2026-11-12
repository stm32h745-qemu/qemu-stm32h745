/*
 * STM32G4 general purpose / basic timer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_TIMER_STM32G4_TIM_H
#define HW_TIMER_STM32G4_TIM_H

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_STM32G4_TIM "stm32g4-tim"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4TimState, STM32G4_TIM)

struct Stm32g4TimState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[4];            /* one line per vector (single line timers use 0) */
    qemu_irq trgo;              /* master mode output */
    qemu_irq dma_update;        /* update DMA request pulse */
    QEMUTimer *timer;

    uint32_t regs[0x400 / 4];
    uint32_t arr_shadow, psc_shadow;    /* values in the preload registers */
    uint32_t arr_active, psc_active;    /* values in use by the counter */

    bool running;
    int64_t epoch_t;            /* virtual time at which the counter was epoch_cnt */
    uint32_t epoch_cnt;
    bool cc_warned;

    uint32_t width;             /* 16 or 32 bit counter */
    uint32_t name_id;
    uint64_t (*clk_hz)(void *opaque);
    void *clk_opaque;
};

#endif
