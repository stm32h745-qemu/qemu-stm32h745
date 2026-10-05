/*
 * STM32H745 SoC model, Cortex-M7 domain.
 *
 * Memory map and interrupt numbers from RM0399 and the STM32H745 datasheet.
 * Modeled: memories, Cortex-M7 (150 IRQs), RCC/PWR/flash interface/HSEM
 * (stm32h7-sysctrl), USART1-3/6, UART4/5/7/8 and LPUART1 (USART v2 models
 * shared with STM32L4), GPIOA-K, FDCAN1/2 with the shared message RAM (SLCAN chardevs
 * "fdcan1"/"fdcan2"). Everything else is an unimplemented device that logs
 * accesses (-d unimp), which is how to find what to model next.
 *
 * Not modeled: the Cortex-M4 domain, caches and their timing, DMA, timers,
 * QUADSPI controller (its memory-mapped window is RAM), FMC, USB, RTC, IWDG.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "system/system.h"
#include "chardev/char.h"
#include "hw/arm/stm32h745_soc.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/misc/unimp.h"

#define ITCM_BASE     0x00000000
#define FLASH_BASE    0x08000000
#define FLASH_SIZE    (2 * MiB)
#define DTCM_BASE     0x20000000
#define AXISRAM_BASE  0x24000000
#define SRAM1_BASE    0x30000000
#define SRAM2_BASE    0x30020000
#define SRAM3_BASE    0x30040000
#define SRAM4_BASE    0x38000000
#define BKPSRAM_BASE  0x38800000
#define QSPI_MEM_BASE 0x90000000
#define QSPI_MEM_SIZE (16 * MiB)

#define RCC_BASE      0x58024400
#define PWR_BASE      0x58024800
#define FLASH_IF_BASE 0x52002000
#define HSEM_BASE     0x58026400
#define FDCAN_RAM_BASE 0x4000AC00
#define FDCAN_RAM_SIZE (10 * KiB)

#define LPUART1_BASE  0x58000C00
#define LPUART1_IRQ   142
#define HSEM1_IRQ     125
#define HSEM2_IRQ     126

static const hwaddr usart_addr[STM32H745_NUM_USARTS] = {
    0x40011000, 0x40004400, 0x40004800, 0x40004C00,   /* USART1-3, UART4 */
    0x40005000, 0x40011400, 0x40007800, 0x40007C00,   /* UART5, USART6, UART7-8 */
};
static const int usart_irq[STM32H745_NUM_USARTS] = { 37, 38, 39, 52, 53, 71, 82, 83 };
static const hwaddr fdcan_addr[STM32H745_NUM_FDCANS] = { 0x4000A000, 0x4000A400 };
static const int fdcan_irq[STM32H745_NUM_FDCANS][2] = { { 19, 21 }, { 20, 22 } };

static void ram(Stm32h745SocState *s, MemoryRegion *mr, const char *name, hwaddr base,
                uint64_t size, Error **errp)
{
    if (memory_region_init_ram(mr, OBJECT(s), name, size, errp)) {
        memory_region_add_subregion(get_system_memory(), base, mr);
    }
}

static void stm32h745_soc_init(Object *obj)
{
    Stm32h745SocState *s = STM32H745_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "sysctrl", &s->sysctrl, TYPE_STM32H7_SYSCTRL);
    for (int i = 0; i < STM32H745_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i], TYPE_STM32L4X5_USART);
    }
    object_initialize_child(obj, "lpuart1", &s->lpuart1, TYPE_STM32L4X5_LPUART);
    for (int i = 0; i < STM32H745_NUM_GPIOS; i++) {
        object_initialize_child(obj, "gpio[*]", &s->gpio[i], TYPE_STM32L4X5_GPIO);
    }
    for (int i = 0; i < STM32H745_NUM_FDCANS; i++) {
        object_initialize_child(obj, "fdcan[*]", &s->fdcan[i], TYPE_STM32H7_FDCAN);
    }
    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = clock_new(obj, "refclk");
    s->pclk = clock_new(obj, "pclk");
}

static void stm32h745_soc_realize(DeviceState *dev, Error **errp)
{
    Stm32h745SocState *s = STM32H745_SOC(dev);
    MemoryRegion *sysmem = get_system_memory();
    DeviceState *armv7m;
    SysBusDevice *busdev;

    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);
    clock_set_mul_div(s->pclk, 2, 1);
    clock_set_source(s->pclk, s->sysclk);

    /* Memories. Flash is RAM so firmware can program it through the flash
     * interface model (erase via FLASH_CR; programming writes land directly). */
    ram(s, &s->flash, "stm32h745.flash", FLASH_BASE, FLASH_SIZE, errp);
    ram(s, &s->itcm, "stm32h745.itcm", ITCM_BASE, 64 * KiB, errp);
    ram(s, &s->dtcm, "stm32h745.dtcm", DTCM_BASE, 128 * KiB, errp);
    ram(s, &s->axisram, "stm32h745.axisram", AXISRAM_BASE, 512 * KiB, errp);
    ram(s, &s->sram1, "stm32h745.sram1", SRAM1_BASE, 128 * KiB, errp);
    ram(s, &s->sram2, "stm32h745.sram2", SRAM2_BASE, 128 * KiB, errp);
    ram(s, &s->sram3, "stm32h745.sram3", SRAM3_BASE, 32 * KiB, errp);
    ram(s, &s->sram4, "stm32h745.sram4", SRAM4_BASE, 64 * KiB, errp);
    ram(s, &s->bkpsram, "stm32h745.bkpsram", BKPSRAM_BASE, 4 * KiB, errp);
    ram(s, &s->qspi_mem, "stm32h745.qspi-mapped", QSPI_MEM_BASE, QSPI_MEM_SIZE, errp);
    ram(s, &s->fdcan_ram, "stm32h745.fdcan-ram", FDCAN_RAM_BASE, FDCAN_RAM_SIZE, errp);
    if (*errp) {
        return;
    }

    /* Cortex-M7, booting from BOOT_CM7_ADD0 = 0x0800 (flash) */
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 150);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m7"));
    qdev_prop_set_uint32(armv7m, "init-nsvtor", FLASH_BASE);
    qdev_prop_set_uint32(armv7m, "mpu-ns-regions", 16);    /* H745 M7: 16 MPU regions */
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(sysmem), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    /* RCC, PWR, flash interface, HSEM */
    object_property_set_link(OBJECT(&s->sysctrl), "flash", OBJECT(&s->flash), &error_abort);
    busdev = SYS_BUS_DEVICE(&s->sysctrl);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    sysbus_mmio_map(busdev, 0, RCC_BASE);
    sysbus_mmio_map(busdev, 1, PWR_BASE);
    sysbus_mmio_map(busdev, 2, FLASH_IF_BASE);
    sysbus_mmio_map(busdev, 3, HSEM_BASE);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, HSEM1_IRQ));

    /* USARTs: serial_hd(0..7) = USART1, USART2, USART3, UART4, UART5,
     * USART6, UART7, UART8 */
    for (int i = 0; i < STM32H745_NUM_USARTS; i++) {
        DeviceState *u = DEVICE(&s->usart[i]);

        qdev_prop_set_chr(u, "chardev", serial_hd(i));
        qdev_connect_clock_in(u, "clk", s->pclk);
        busdev = SYS_BUS_DEVICE(u);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    }

    /* LPUART1 (the NUCLEO-H745ZI-Q console): serial_hd(8) */
    {
        DeviceState *u = DEVICE(&s->lpuart1);

        qdev_prop_set_chr(u, "chardev", serial_hd(STM32H745_NUM_USARTS));
        qdev_connect_clock_in(u, "clk", s->pclk);
        busdev = SYS_BUS_DEVICE(u);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, LPUART1_BASE);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, LPUART1_IRQ));
    }

    /* GPIOA..K */
    for (int i = 0; i < STM32H745_NUM_GPIOS; i++) {
        DeviceState *g = DEVICE(&s->gpio[i]);
        g_autofree char *name = g_strdup_printf("%c", 'A' + i);

        qdev_prop_set_string(g, "name", name);
        qdev_prop_set_uint32(g, "mode-reset", i == 0 ? 0xABFFFFFF : i == 1 ? 0xFFFFFEBF : 0xFFFFFFFF);
        qdev_connect_clock_in(g, "clk", s->pclk);
        busdev = SYS_BUS_DEVICE(g);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, 0x58020000 + i * 0x400);
    }

    /* FDCAN1/2: bus = chardev "fdcan1"/"fdcan2" (SLCAN), if given */
    for (int i = 0; i < STM32H745_NUM_FDCANS; i++) {
        DeviceState *f = DEVICE(&s->fdcan[i]);
        g_autofree char *id = g_strdup_printf("fdcan%d", i + 1);
        Chardev *chr = qemu_chr_find(id);

        if (chr) {
            qdev_prop_set_chr(f, "chardev", chr);
        }
        object_property_set_link(OBJECT(f), "msg-ram", OBJECT(&s->fdcan_ram), &error_abort);
        busdev = SYS_BUS_DEVICE(f);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, fdcan_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, fdcan_irq[i][0]));
        sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(armv7m, fdcan_irq[i][1]));
    }

    /* Not modeled yet: accesses are logged with -d unimp */
    create_unimplemented_device("TIM2-7",      0x40000000, 0x1800);
    create_unimplemented_device("TIM12-14",    0x40001800, 0x0C00);
    create_unimplemented_device("LPTIM1",      0x40002400, 0x400);
    create_unimplemented_device("SPI2",        0x40003800, 0x400);
    create_unimplemented_device("SPI3",        0x40003C00, 0x400);
    create_unimplemented_device("SPDIFRX",     0x40004000, 0x400);
    create_unimplemented_device("I2C1",        0x40005400, 0x400);
    create_unimplemented_device("I2C2",        0x40005800, 0x400);
    create_unimplemented_device("I2C3",        0x40005C00, 0x400);
    create_unimplemented_device("HDMI-CEC",    0x40006C00, 0x400);
    create_unimplemented_device("DAC1",        0x40007400, 0x400);
    create_unimplemented_device("CRS",         0x40008400, 0x400);
    create_unimplemented_device("SWPMI",       0x40008800, 0x400);
    create_unimplemented_device("OPAMP",       0x40009000, 0x400);
    create_unimplemented_device("MDIOS",       0x40009400, 0x400);
    create_unimplemented_device("FDCAN-CCU",   0x4000A800, 0x400);
    create_unimplemented_device("TIM1",        0x40010000, 0x400);
    create_unimplemented_device("TIM8",        0x40010400, 0x400);
    create_unimplemented_device("SPI1",        0x40013000, 0x400);
    create_unimplemented_device("SPI4",        0x40013400, 0x400);
    create_unimplemented_device("TIM15-17",    0x40014000, 0x0C00);
    create_unimplemented_device("SPI5",        0x40015000, 0x400);
    create_unimplemented_device("SAI1-3",      0x40015800, 0x0C00);
    create_unimplemented_device("DFSDM1",      0x40017000, 0x400);
    create_unimplemented_device("HRTIM",       0x40017400, 0x400);
    create_unimplemented_device("DMA1",        0x40020000, 0x400);
    create_unimplemented_device("DMA2",        0x40020400, 0x400);
    create_unimplemented_device("DMAMUX1",     0x40020800, 0x400);
    create_unimplemented_device("ADC12",       0x40022000, 0x400);
    create_unimplemented_device("ART",         0x40024400, 0x400);
    create_unimplemented_device("ETH",         0x40028000, 0x2400);
    create_unimplemented_device("USB1-OTG-HS", 0x40040000, 0x40000);
    create_unimplemented_device("USB2-OTG-FS", 0x40080000, 0x40000);
    create_unimplemented_device("DCMI",        0x48020000, 0x400);
    create_unimplemented_device("RNG",         0x48021800, 0x400);
    create_unimplemented_device("SDMMC2",      0x48022400, 0x400);
    create_unimplemented_device("RAMECC2",     0x48023000, 0x400);
    create_unimplemented_device("LTDC",        0x50001000, 0x1000);
    create_unimplemented_device("WWDG1",       0x50003000, 0x400);
    create_unimplemented_device("MDMA",        0x52000000, 0x1000);
    create_unimplemented_device("DMA2D",       0x52001000, 0x1000);
    create_unimplemented_device("JPEG",        0x52003000, 0x1000);
    create_unimplemented_device("FMC",         0x52004000, 0x1000);
    create_unimplemented_device("QUADSPI",     0x52005000, 0x1000);
    create_unimplemented_device("SDMMC1",      0x52007000, 0x1000);
    create_unimplemented_device("RAMECC1",     0x52009000, 0x1000);
    create_unimplemented_device("EXTI",        0x58000000, 0x400);
    create_unimplemented_device("SYSCFG",      0x58000400, 0x400);
    create_unimplemented_device("SPI6",        0x58001400, 0x400);
    create_unimplemented_device("I2C4",        0x58001C00, 0x400);
    create_unimplemented_device("LPTIM2-5",    0x58002400, 0x1000);
    create_unimplemented_device("COMP",        0x58003800, 0x400);
    create_unimplemented_device("VREFBUF",     0x58003C00, 0x400);
    create_unimplemented_device("RTC",         0x58004000, 0x400);
    create_unimplemented_device("IWDG1",       0x58004800, 0x400);
    create_unimplemented_device("IWDG2",       0x58004C00, 0x400);
    create_unimplemented_device("SAI4",        0x58005400, 0x400);
    create_unimplemented_device("CRC",         0x58024C00, 0x400);
    create_unimplemented_device("BDMA",        0x58025400, 0x400);
    create_unimplemented_device("DMAMUX2",     0x58025800, 0x400);
    create_unimplemented_device("ADC3",        0x58026000, 0x400);
    create_unimplemented_device("RAMECC3",     0x58027000, 0x400);
    create_unimplemented_device("DBGMCU",      0x5C001000, 0x400);
}

static void stm32h745_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32h745_soc_realize;
    dc->user_creatable = false;
}

static const TypeInfo stm32h745_soc_info = {
    .name = TYPE_STM32H745_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h745SocState),
    .instance_init = stm32h745_soc_init,
    .class_init = stm32h745_soc_class_init,
};

static void stm32h745_soc_register(void)
{
    type_register_static(&stm32h745_soc_info);
}
type_init(stm32h745_soc_register)
