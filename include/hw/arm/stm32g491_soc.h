/*
 * STM32G491 SoC model (Cortex-M4F).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_STM32G491_SOC_H
#define HW_ARM_STM32G491_SOC_H

#include "hw/arm/armv7m.h"
#include "hw/char/stm32h7_usart.h"
#include "hw/gpio/stm32g4_gpio.h"
#include "hw/misc/stm32g4_sysctrl.h"
#include "hw/misc/stm32g4_exti.h"
#include "hw/net/stm32h7_fdcan.h"
#include "hw/adc/stm32g4_adc.h"
#include "hw/timer/stm32g4_tim.h"
#include "hw/clock.h"
#include "qom/object.h"

#define TYPE_STM32G491_SOC "stm32g491-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g491SocState, STM32G491_SOC)

#define STM32G491_NUM_USARTS 6      /* USART1-3, UART4-5, LPUART1 */
#define STM32G491_NUM_GPIOS  7      /* GPIOA-G */
#define STM32G491_NUM_FDCANS 2
#define STM32G491_NUM_TIMS   10

struct Stm32g491SocState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;
    Stm32g4SysctrlState sysctrl;
    Stm32g4ExtiState exti;
    Stm32g4GpioState gpio[STM32G491_NUM_GPIOS];
    Stm32h7UsartState usart[STM32G491_NUM_USARTS];
    Stm32h7FdcanState fdcan[STM32G491_NUM_FDCANS];
    Stm32g4AdcState adc12, adc3;
    Stm32g4TimState tim[STM32G491_NUM_TIMS];
    DeviceState *dma[2];
    DeviceState *dmamux;

    MemoryRegion sram, ccm_alias, flash_alias, fdcan_ram, sysmem, dbgmcu;
    Clock *refclk;          /* SysTick reference clock (HCLK / 8) */
    Clock *pclk;            /* unused input clock of the USART models */

    uint32_t hse_hz;
    uint32_t vref_mv;
    uint32_t uid_seed;
    uint32_t dbgmcu_cr;
    char *adc_mv;           /* analog inputs: "<adc>.<ch>=<mV>,..." */
    char *gpio_in;          /* initial pin levels: "<port><pin>=<0|1>;..." */
    char *flash_image;      /* ':' separated list: file.hex or file.bin@offset */
};

#endif
