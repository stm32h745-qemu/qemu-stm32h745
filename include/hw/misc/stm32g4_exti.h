/*
 * STM32G4 EXTI and SYSCFG.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_STM32G4_EXTI_H
#define HW_MISC_STM32G4_EXTI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32G4_EXTI "stm32g4-exti"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4ExtiState, STM32G4_EXTI)

#define STM32G4_EXTI_PORTS 7        /* GPIOA..GPIOG */
#define STM32G4_EXTI_IRQS  7        /* line 0..4 each, 5-9, 10-15 */

struct Stm32g4ExtiState {
    SysBusDevice parent_obj;
    MemoryRegion exti_mmio;
    MemoryRegion syscfg_mmio;
    qemu_irq irq[STM32G4_EXTI_IRQS];

    uint32_t rtsr[2], ftsr[2], swier[2], pr[2], imr[2], emr[2];
    uint32_t syscfg[0x40 / 4];
    uint16_t port_level[STM32G4_EXTI_PORTS];    /* pad levels of every GPIO port */
    uint16_t line_level;                        /* level seen by each GPIO line */
};

#endif
