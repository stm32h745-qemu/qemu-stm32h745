/*
 * STM32H745 SoC model, Cortex-M7 domain.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_STM32H745_SOC_H
#define HW_ARM_STM32H745_SOC_H

#include "hw/arm/armv7m.h"
#include "hw/char/stm32l4x5_usart.h"
#include "hw/gpio/stm32l4x5_gpio.h"
#include "hw/misc/stm32h7_sysctrl.h"
#include "hw/net/stm32h7_fdcan.h"
#include "hw/clock.h"
#include "qom/object.h"

#define TYPE_STM32H745_SOC "stm32h745-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h745SocState, STM32H745_SOC)

#define STM32H745_NUM_USARTS 8
#define STM32H745_NUM_GPIOS  11
#define STM32H745_NUM_FDCANS 2

struct Stm32h745SocState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;
    Stm32h7SysctrlState sysctrl;
    Stm32l4x5UsartBaseState usart[STM32H745_NUM_USARTS];
    Stm32l4x5UsartBaseState lpuart1;
    Stm32l4x5GpioState gpio[STM32H745_NUM_GPIOS];
    Stm32h7FdcanState fdcan[STM32H745_NUM_FDCANS];

    MemoryRegion itcm, dtcm, flash, axisram, sram1, sram2, sram3, sram4, bkpsram;
    MemoryRegion qspi_mem, fdcan_ram;

    Clock *sysclk;      /* M7 core clock (HCLK) */
    Clock *refclk;      /* SysTick external reference (HCLK / 8) */
    Clock *pclk;        /* peripheral kernel clock for USART/GPIO models */

    uint32_t sysclk_hz;
};

#endif
