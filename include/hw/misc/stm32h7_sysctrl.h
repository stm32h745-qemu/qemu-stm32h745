/*
 * STM32H7 (H745/H747/H755/H757) system control: RCC, PWR, embedded flash
 * interface and hardware semaphores (HSEM).
 *
 * A behavioral model at the level firmware needs
 * to boot and run: registers latch, every "ready"/"status" bit firmware
 * polls follows its enable (oscillators, PLLs, clock switch, voltage
 * scaling), the flash interface unlocks, erases and reports idle, and HSEM
 * implements 1-step/2-step locking with per-core interrupts. No clock tree
 * arithmetic: the SoC's clocks are fixed by board properties.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_STM32H7_SYSCTRL_H
#define HW_MISC_STM32H7_SYSCTRL_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32H7_SYSCTRL "stm32h7-sysctrl"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7SysctrlState, STM32H7_SYSCTRL)

#define STM32H7_HSEM_COUNT 32

struct Stm32h7SysctrlState {
    SysBusDevice parent_obj;

    MemoryRegion rcc_mmio;
    MemoryRegion pwr_mmio;
    MemoryRegion flash_mmio;
    MemoryRegion hsem_mmio;

    uint32_t rcc[0x400 / 4];
    uint32_t pwr[0x400 / 4];
    uint32_t flash[0x1000 / 4];
    uint8_t flash_key_state[2];
    uint8_t opt_key_state;

    /* HSEM: R[i] holds LOCK | COREID | PROCID; interrupt state per core */
    uint32_t hsem_r[STM32H7_HSEM_COUNT];
    uint32_t hsem_ier[2];
    uint32_t hsem_isr[2];
    uint32_t hsem_keyr;
    qemu_irq hsem_irq[2];  /* 0 = CM7 (HSEM1), 1 = CM4 (HSEM2) */

    /* Flash array (bank 1 then bank 2) for erase operations */
    MemoryRegion *flash_mem;
    uint32_t flash_size;

    /* Option bytes (OPTSR_CUR/PRG reset value) */
    uint32_t optsr;
    uint32_t boot_cm4_add;

    /* Raised when firmware forces the CM4 to boot (RCC_GCR.BOOT_C2) */
    qemu_irq cm4_boot;
};

/* Clock of APB bus apb (1 = D2 APB1, 2 = D2 APB2, 3 = D1 APB3, 4 = D3 APB4)
 * for core clock core_hz, from the prescalers firmware set in RCC */
uint64_t stm32h7_sysctrl_apb_hz(Stm32h7SysctrlState *s, int apb, uint64_t core_hz);

#endif
