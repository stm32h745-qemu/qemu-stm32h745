/*
 * Machine "stm32g491-generic": an STM32G491 (Cortex-M4F, 512 KB flash) with
 * nothing wired to it beyond the chip itself; pin assignments, analog levels
 * and the oscillator are given on the command line.
 *
 * Properties:
 *   hse-hz=<Hz>        HSE oscillator frequency (default 8000000)
 *   flash-image=<list> ':' separated images loaded into the flash: file.hex
 *                      (Intel HEX, absolute addresses) or file.bin@<offset>;
 *                      loaded once at start, the flash then lives its own life
 *                      (erase/program by firmware survive a system reset)
 *   adc-mv=<list>      initial analog inputs "<adc>.<channel>=<mV>;..." (; or , separated)
 *   gpio-in=<list>     initial levels driven onto pins "<port><pin>=<0|1>;..."
 *   vref-mv=<mV>       ADC reference (VDDA), default 3300
 *   uid-seed=<n>       varies the 96-bit unique ID
 * -kernel <elf> works too (loaded at the flash base).
 * Serial ports, in order: USART1, USART2, USART3, UART4, UART5, LPUART1.
 * CAN: -chardev socket,id=fdcan1,... (and fdcan2) carry SLCAN text frames.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/arm/boot.h"
#include "hw/arm/stm32g491_soc.h"

#define TYPE_STM32G491_MACHINE MACHINE_TYPE_NAME("stm32g491-generic")
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g491MachineState, STM32G491_MACHINE)

struct Stm32g491MachineState {
    MachineState parent_obj;
    uint32_t hse_hz;
    uint32_t vref_mv;
    uint32_t uid_seed;
    char *flash_image;
    char *adc_mv;
    char *gpio_in;
};

static void stm32g491_init(MachineState *machine)
{
    Stm32g491MachineState *m = STM32G491_MACHINE(machine);
    DeviceState *dev = qdev_new(TYPE_STM32G491_SOC);

    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_prop_set_uint32(dev, "hse-hz", m->hse_hz);
    qdev_prop_set_uint32(dev, "vref-mv", m->vref_mv);
    qdev_prop_set_uint32(dev, "uid-seed", m->uid_seed);
    if (m->flash_image) {
        qdev_prop_set_string(dev, "flash-image", m->flash_image);
    }
    if (m->adc_mv) {
        qdev_prop_set_string(dev, "adc-mv", m->adc_mv);
    }
    if (m->gpio_in) {
        qdev_prop_set_string(dev, "gpio-in", m->gpio_in);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    /* also registers the CPU reset handler, so it must always be called */
    armv7m_load_kernel(STM32G491_SOC(dev)->armv7m.cpu, machine->kernel_filename,
                       0x08000000, 512 * 1024);
}

static void get_u32(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    uint32_t *p = (uint32_t *)((char *)obj + (uintptr_t)opaque);

    visit_type_uint32(v, name, p, errp);
}

static void set_u32(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    uint32_t *p = (uint32_t *)((char *)obj + (uintptr_t)opaque);

    visit_type_uint32(v, name, p, errp);
}

static char *get_flash_image(Object *obj, Error **errp)
{
    return g_strdup(STM32G491_MACHINE(obj)->flash_image);
}

static void set_flash_image(Object *obj, const char *v, Error **errp)
{
    Stm32g491MachineState *m = STM32G491_MACHINE(obj);

    g_free(m->flash_image);
    m->flash_image = g_strdup(v);
}

static char *get_adc_mv(Object *obj, Error **errp)
{
    return g_strdup(STM32G491_MACHINE(obj)->adc_mv);
}

static void set_adc_mv(Object *obj, const char *v, Error **errp)
{
    Stm32g491MachineState *m = STM32G491_MACHINE(obj);

    g_free(m->adc_mv);
    m->adc_mv = g_strdup(v);
}

static char *get_gpio_in(Object *obj, Error **errp)
{
    return g_strdup(STM32G491_MACHINE(obj)->gpio_in);
}

static void set_gpio_in(Object *obj, const char *v, Error **errp)
{
    Stm32g491MachineState *m = STM32G491_MACHINE(obj);

    g_free(m->gpio_in);
    m->gpio_in = g_strdup(v);
}

static void machine_instance_init(Object *obj)
{
    Stm32g491MachineState *m = STM32G491_MACHINE(obj);

    m->hse_hz = 8000000;
    m->vref_mv = 3300;
}

static const char * const valid_cpu_types[] = { ARM_CPU_TYPE_NAME("cortex-m4"), NULL };

static void machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "STM32G491 (Cortex-M4F, 512 KB flash), no board wiring";
    mc->init = stm32g491_init;
    mc->valid_cpu_types = valid_cpu_types;
    object_class_property_add(oc, "hse-hz", "uint32", get_u32, set_u32, NULL,
                              (void *)offsetof(Stm32g491MachineState, hse_hz));
    object_class_property_set_description(oc, "hse-hz", "HSE oscillator frequency in Hz");
    object_class_property_add(oc, "vref-mv", "uint32", get_u32, set_u32, NULL,
                              (void *)offsetof(Stm32g491MachineState, vref_mv));
    object_class_property_set_description(oc, "vref-mv", "ADC reference voltage in mV");
    object_class_property_add(oc, "uid-seed", "uint32", get_u32, set_u32, NULL,
                              (void *)offsetof(Stm32g491MachineState, uid_seed));
    object_class_property_set_description(oc, "uid-seed", "varies the 96-bit unique ID");
    object_class_property_add_str(oc, "flash-image", get_flash_image, set_flash_image);
    object_class_property_set_description(oc, "flash-image",
        "':' separated flash images: file.hex or file.bin@offset");
    object_class_property_add_str(oc, "gpio-in", get_gpio_in, set_gpio_in);
    object_class_property_set_description(oc, "gpio-in",
        "initial levels driven onto input pins: <port><pin>=<0|1>;... (e.g. C0=1;A8=1)");
    object_class_property_add_str(oc, "adc-mv", get_adc_mv, set_adc_mv);
    object_class_property_set_description(oc, "adc-mv",
        "initial analog inputs: <adc>.<channel>=<mV>;... (use ; on the -machine line)");
}

static const TypeInfo stm32g491_machine_info = {
    .name = TYPE_STM32G491_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Stm32g491MachineState),
    .instance_init = machine_instance_init,
    .class_init = machine_class_init,
};

static void stm32g491_machine_register(void)
{
    type_register_static(&stm32g491_machine_info);
}
type_init(stm32g491_machine_register)
