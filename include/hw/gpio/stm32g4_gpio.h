/*
 * STM32G4 GPIO port with externally drivable inputs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_GPIO_STM32G4_GPIO_H
#define HW_GPIO_STM32G4_GPIO_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32G4_GPIO "stm32g4-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4GpioState, STM32G4_GPIO)

#define STM32G4_GPIO_PINS 16

struct Stm32g4GpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t moder, otyper, ospeedr, pupdr, odr, lckr, afrl, afrh;
    uint32_t mode_reset, ospeed_reset, pupd_reset;

    uint16_t ext_level;     /* level driven onto the pin from outside */
    uint16_t ext_driven;    /* which pins are driven from outside (else pulled/floating) */
    uint16_t pad;           /* resulting pad level, as seen by EXTI */
    qemu_irq pad_out[STM32G4_GPIO_PINS];

    char *name;             /* "A".."G" */
};

/* Drive pin (0-15) from outside: level 0/1, or -1 to release it */
void stm32g4_gpio_set_input(Stm32g4GpioState *s, int pin, int level);

#endif
