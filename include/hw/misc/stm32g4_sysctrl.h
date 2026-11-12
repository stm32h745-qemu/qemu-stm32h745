/*
 * STM32G4 system control: RCC, PWR and the embedded flash (array and
 * interface), for the STM32G491 machine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_STM32G4_SYSCTRL_H
#define HW_MISC_STM32G4_SYSCTRL_H

#include "hw/sysbus.h"
#include "hw/clock.h"
#include "qom/object.h"

#define TYPE_STM32G4_SYSCTRL "stm32g4-sysctrl"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4SysctrlState, STM32G4_SYSCTRL)

/* Clocks a peripheral model can ask for (all derived from RCC registers) */
typedef enum {
    STM32G4_CLK_SYSCLK,
    STM32G4_CLK_HCLK,
    STM32G4_CLK_PCLK1,
    STM32G4_CLK_PCLK2,
    STM32G4_CLK_TIMCLK1,    /* APB1 timer clock (x2 if the APB1 prescaler > 1) */
    STM32G4_CLK_TIMCLK2,
    STM32G4_CLK_ADC12,      /* ADC12 kernel clock (CCIPR.ADC12SEL) */
    STM32G4_CLK_ADC345,
} Stm32g4Clk;

struct Stm32g4SysctrlState {
    SysBusDevice parent_obj;

    MemoryRegion rcc_mmio;
    MemoryRegion pwr_mmio;
    MemoryRegion flash_if_mmio;
    MemoryRegion flash_mem;         /* the array, a ROM device: writes go through
                                       the flash interface programming rules */
    Clock *hclk;                    /* output: core clock */
    qemu_irq flash_irq;

    uint32_t rcc[0x100 / 4];
    uint32_t pwr[0x100 / 4];
    uint32_t fl[0x80 / 4];
    uint32_t sws;                   /* clock actually used as SYSCLK */
    uint8_t key_state, optkey_state;

    /* Double-word programming in progress */
    bool pg_pending;
    uint32_t pg_addr;               /* offset of the double word */
    uint32_t pg_word[2];
    uint8_t pg_have;

    uint32_t flash_size;
    uint32_t hse_hz;
    uint32_t optr;
};

uint64_t stm32g4_sysctrl_hz(Stm32g4SysctrlState *s, Stm32g4Clk clk);
uint8_t *stm32g4_sysctrl_flash_ptr(Stm32g4SysctrlState *s);

#endif
