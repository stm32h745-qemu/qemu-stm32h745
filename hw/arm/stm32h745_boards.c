/*
 * Machines on the STM32H745 SoC model (Cortex-M7 domain):
 *  - stm32h745:     a bare STM32H745 board. The core clock is a machine
 *                   property (sysclk-hz, default 480 MHz) so it can match the
 *                   clock tree the firmware configures.
 *  - nucleo-h745zi: ST NUCLEO-H745ZI-Q, M7 at 480 MHz.
 *
 * -kernel loads an ELF (or a raw image at 0x08000000); further images (for
 * example a bootloader plus an application) with -device loader.
 * Serial ports, in order: USART1, USART2, USART3, UART4, UART5, USART6,
 * UART7, UART8, LPUART1.
 * CAN: -chardev socket,id=fdcan1,... (and fdcan2) carry SLCAN text frames;
 * see docs/system/arm/stm32h745.rst.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/boards.h"
#include "hw/qdev-clock.h"
#include "hw/arm/boot.h"
#include "hw/arm/stm32h745_soc.h"

#define TYPE_STM32H745_MACHINE MACHINE_TYPE_NAME("stm32h745")
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h745MachineState, STM32H745_MACHINE)

struct Stm32h745MachineState {
    MachineState parent_obj;
    uint32_t sysclk_hz;
};

static void stm32h745_board_init(MachineState *machine, uint32_t sysclk_hz)
{
    DeviceState *dev;
    Clock *sysclk = clock_new(OBJECT(machine), "SYSCLK");

    clock_set_hz(sysclk, sysclk_hz);
    dev = qdev_new(TYPE_STM32H745_SOC);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    armv7m_load_kernel(STM32H745_SOC(dev)->armv7m.cpu, machine->kernel_filename,
                       0x08000000, 2 * 1024 * 1024);
}

static void stm32h745_init(MachineState *machine)
{
    stm32h745_board_init(machine, STM32H745_MACHINE(machine)->sysclk_hz);
}

static void stm32h745_get_sysclk(Object *obj, Visitor *v, const char *name, void *opaque,
                                 Error **errp)
{
    visit_type_uint32(v, name, &STM32H745_MACHINE(obj)->sysclk_hz, errp);
}

static void stm32h745_set_sysclk(Object *obj, Visitor *v, const char *name, void *opaque,
                                 Error **errp)
{
    uint32_t hz;

    if (!visit_type_uint32(v, name, &hz, errp)) {
        return;
    }
    if (hz == 0 || hz > 480 * 1000 * 1000) {
        error_setg(errp, "sysclk-hz must be between 1 and 480000000");
        return;
    }
    STM32H745_MACHINE(obj)->sysclk_hz = hz;
}

static void stm32h745_machine_instance_init(Object *obj)
{
    STM32H745_MACHINE(obj)->sysclk_hz = 480 * 1000 * 1000;
}

static const char * const valid_cpu_types[] = { ARM_CPU_TYPE_NAME("cortex-m7"), NULL };

static void stm32h745_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "STM32H745 (Cortex-M7 domain), core clock set by sysclk-hz";
    mc->init = stm32h745_init;
    mc->valid_cpu_types = valid_cpu_types;
    object_class_property_add(oc, "sysclk-hz", "uint32", stm32h745_get_sysclk,
                              stm32h745_set_sysclk, NULL, NULL);
    object_class_property_set_description(oc, "sysclk-hz",
                                          "M7 core clock in Hz (default 480000000)");
}

static const TypeInfo stm32h745_machine_info = {
    .name = TYPE_STM32H745_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Stm32h745MachineState),
    .instance_init = stm32h745_machine_instance_init,
    .class_init = stm32h745_machine_class_init,
};

static void stm32h745_machine_register(void)
{
    type_register_static(&stm32h745_machine_info);
}
type_init(stm32h745_machine_register)

static void nucleo_h745zi_init(MachineState *machine)
{
    stm32h745_board_init(machine, 480 * 1000 * 1000);
}

static void nucleo_h745zi_machine_init(MachineClass *mc)
{
    mc->desc = "ST NUCLEO-H745ZI-Q (STM32H745, Cortex-M7 domain)";
    mc->init = nucleo_h745zi_init;
    mc->valid_cpu_types = valid_cpu_types;
}
DEFINE_MACHINE("nucleo-h745zi", nucleo_h745zi_machine_init)
