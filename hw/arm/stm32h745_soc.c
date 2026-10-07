/*
 * STM32H745 SoC model, Cortex-M7 domain.
 *
 * Memory map and interrupt numbers from RM0399 and the STM32H745 datasheet.
 * Modeled: memories, Cortex-M7 (150 IRQs), RCC/PWR/flash interface/HSEM
 * (stm32h7-sysctrl), USART1-3/6 and UART4/5/7/8 (stm32h7-usart: DMA requests,
 * IDLE, line-rate reception from the RCC prescalers), LPUART1 (STM32L4 model),
 * DMA1/DMA2 and DMAMUX1, GPIOA-K, FDCAN1/2 with the shared message RAM (SLCAN chardevs
 * "fdcan1"/"fdcan2"), QUADSPI with a 32 MiB NOR flash (backing image: the first
 * -drive if=mtd), ADC1-3 (inputs in mV from the adc-mv property), DBGMCU
 * IDCODE (STM32H745, revision V), RTC, RNG, IWDG1 (resets the machine on
 * timeout), USB OTG HS/FS with no cable attached, I2C1-4 (buses "i2c1".."i2c4"
 * for -device ...,bus=i2cN). Everything else is an unimplemented device that logs
 * accesses (-d unimp), which is how to find what to model next.
 *
 * Not modeled: the Cortex-M4 domain, caches and their timing, BDMA/MDMA, timers,
 * FMC, USB traffic.
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
#include "system/blockdev.h"
#include "system/block-backend.h"

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
#define SYSMEM_BASE   0x1FF00000    /* system memory: boot ROM, UID, flash size */
#define SYSMEM_SIZE   (128 * KiB)
#define UID_OFF       0x1E800       /* 96-bit unique ID */
#define FLASHSIZE_OFF 0x1E880       /* flash size in KiB, 16 bits */
#define QSPI_MEM_BASE 0x90000000
#define QSPI_BASE     0x52005000
#define QSPI_IRQ      92

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
static const int usart_apb[STM32H745_NUM_USARTS] = { 2, 1, 1, 1, 1, 2, 1, 1 };
/* DMAMUX1 request IDs (RX; TX is RX + 1) */
static const int usart_dmareq[STM32H745_NUM_USARTS] = { 41, 43, 45, 63, 65, 71, 79, 81 };
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
        object_initialize_child(obj, "usart[*]", &s->usart[i], TYPE_STM32H7_USART);
    }
    object_initialize_child(obj, "lpuart1", &s->lpuart1, TYPE_STM32L4X5_LPUART);
    for (int i = 0; i < STM32H745_NUM_GPIOS; i++) {
        object_initialize_child(obj, "gpio[*]", &s->gpio[i], TYPE_STM32L4X5_GPIO);
    }
    for (int i = 0; i < STM32H745_NUM_FDCANS; i++) {
        object_initialize_child(obj, "fdcan[*]", &s->fdcan[i], TYPE_STM32H7_FDCAN);
    }
    object_initialize_child(obj, "qspi", &s->qspi, TYPE_STM32H7_QSPI);
    object_initialize_child(obj, "adc12", &s->adc12, TYPE_STM32H7_ADC);
    object_initialize_child(obj, "adc3", &s->adc3, TYPE_STM32H7_ADC);
    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = clock_new(obj, "refclk");
    s->pclk = clock_new(obj, "pclk");
}

/* DBGMCU: IDCODE = STM32H745/755 (DEV_ID 0x450), revision V (0x2003) */
static uint64_t dbgmcu_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h745SocState *s = opaque;

    switch (off) {
    case 0x00: return 0x20036450;
    case 0x04: return s->dbgmcu_cr;
    }
    return 0;
}

static void dbgmcu_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h745SocState *s = opaque;

    if (off == 0x04) {
        s->dbgmcu_cr = v;
    }
}

static const MemoryRegionOps dbgmcu_ops = {
    .read = dbgmcu_read,
    .write = dbgmcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* adc-mv: comma-separated "<adc 1-3>.<channel>=<mV>" */
static bool apply_adc_inputs(Stm32h745SocState *s, Error **errp)
{
    g_auto(GStrv) items = NULL;

    if (!s->adc_mv || !*s->adc_mv) {
        return true;
    }
    items = g_strsplit(s->adc_mv, ",", -1);
    for (char **it = items; *it; it++) {
        unsigned adc, ch, mv;

        if (sscanf(*it, "%u.%u=%u", &adc, &ch, &mv) != 3 || adc < 1 || adc > 3 ||
            ch >= STM32H7_ADC_CHANNELS) {
            error_setg(errp, "adc-mv: bad item '%s' (want <adc 1-3>.<ch 0-19>=<mV>)",
                       *it);
            return false;
        }
        if (adc == 3) {
            stm32h7_adc_set_input_mv(&s->adc3, 0, ch, mv);
        } else {
            stm32h7_adc_set_input_mv(&s->adc12, adc - 1, ch, mv);
        }
    }
    return true;
}

typedef struct UsartClk {
    Stm32h745SocState *soc;
    int apb;
} UsartClk;

static uint64_t usart_kernel_hz(void *opaque)
{
    UsartClk *c = opaque;

    return stm32h7_sysctrl_apb_hz(&c->soc->sysctrl, c->apb, clock_get_hz(c->soc->sysclk));
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
    ram(s, &s->fdcan_ram, "stm32h745.fdcan-ram", FDCAN_RAM_BASE, FDCAN_RAM_SIZE, errp);
    if (*errp) {
        return;
    }
    /* Erased flash reads 0xFF (images are loaded over it afterwards) */
    memset(memory_region_get_ram_ptr(&s->flash), 0xFF, FLASH_SIZE);

    /* System memory (read-only): unique ID and flash size, which firmware
     * reads for its serial number and flash geometry. uid-seed varies the ID
     * so several instances differ. */
    if (!memory_region_init_rom(&s->sysmem, OBJECT(s), "stm32h745.sysmem",
                                SYSMEM_SIZE, errp)) {
        return;
    }
    {
        uint8_t *p = memory_region_get_ram_ptr(&s->sysmem);
        uint32_t uid[3] = { 0x00410022 ^ s->uid_seed, 0x3133510D + s->uid_seed,
                            0x30393538 };

        memset(p, 0xFF, SYSMEM_SIZE);
        for (int i = 0; i < 3; i++) {
            stl_le_p(p + UID_OFF + 4 * i, uid[i]);
        }
        stw_le_p(p + FLASHSIZE_OFF, FLASH_SIZE / KiB);
    }
    memory_region_add_subregion(get_system_memory(), SYSMEM_BASE, &s->sysmem);

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

    /* DMA1, DMA2 and DMAMUX1 (outputs 0-7: DMA1 streams, 8-15: DMA2 streams) */
    {
        static const int dma_irq[2][8] = {
            { 11, 12, 13, 14, 15, 16, 17, 47 }, { 56, 57, 58, 59, 60, 68, 69, 70 },
        };
        DeviceState *dma[2];

        for (int d = 0; d < 2; d++) {
            dma[d] = qdev_new("stm32h7-dma");
            object_property_add_child(OBJECT(s), d ? "dma2" : "dma1", OBJECT(dma[d]));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dma[d]), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dma[d]), 0, 0x40020000 + 0x400 * d);
            for (int n = 0; n < 8; n++) {
                sysbus_connect_irq(SYS_BUS_DEVICE(dma[d]), n,
                                   qdev_get_gpio_in(armv7m, dma_irq[d][n]));
            }
        }
        s->dmamux = qdev_new("stm32h7-dmamux");
        object_property_add_child(OBJECT(s), "dmamux1", OBJECT(s->dmamux));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->dmamux), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->dmamux), 0, 0x40020800);
        for (int c = 0; c < 16; c++) {
            qdev_connect_gpio_out_named(s->dmamux, "out", c,
                                        qdev_get_gpio_in_named(dma[c / 8], "req", c % 8));
        }
    }

    /* USARTs: serial_hd(0..7) = USART1, USART2, USART3, UART4, UART5,
     * USART6, UART7, UART8 */
    for (int i = 0; i < STM32H745_NUM_USARTS; i++) {
        DeviceState *u = DEVICE(&s->usart[i]);
        UsartClk *clk = g_new(UsartClk, 1);

        clk->soc = s;
        clk->apb = usart_apb[i];
        s->usart[i].kernel_hz = usart_kernel_hz;
        s->usart[i].kernel_hz_opaque = clk;

        qdev_prop_set_chr(u, "chardev", serial_hd(i));
        qdev_connect_clock_in(u, "clk", s->pclk);
        busdev = SYS_BUS_DEVICE(u);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
        qdev_connect_gpio_out_named(u, "dma-rx", 0,
                                    qdev_get_gpio_in_named(s->dmamux, "req",
                                                           usart_dmareq[i]));
        qdev_connect_gpio_out_named(u, "dma-tx", 0,
                                    qdev_get_gpio_in_named(s->dmamux, "req",
                                                           usart_dmareq[i] + 1));
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

    /* QUADSPI + NOR flash; contents from the first -drive if=mtd, if any */
    {
        DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);

        if (dinfo) {
            qdev_prop_set_drive_err(DEVICE(&s->qspi), "drive",
                                    blk_by_legacy_dinfo(dinfo), &error_abort);
        }
        busdev = SYS_BUS_DEVICE(&s->qspi);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, QSPI_BASE);
        sysbus_mmio_map(busdev, 1, QSPI_MEM_BASE);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, QSPI_IRQ));
    }

    /* ADC1+ADC2 (shared interrupt) and ADC3 */
    qdev_prop_set_uint32(DEVICE(&s->adc12), "num", 2);
    if (!apply_adc_inputs(s, errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(&s->adc12), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(&s->adc3), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adc12), 0, 0x40022000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc12), 0, qdev_get_gpio_in(armv7m, 18));
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adc3), 0, 0x58026000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc3), 0, qdev_get_gpio_in(armv7m, 127));

    memory_region_init_io(&s->dbgmcu, OBJECT(s), &dbgmcu_ops, s, "stm32h745.dbgmcu",
                          0x400);
    memory_region_add_subregion(get_system_memory(), 0x5C001000, &s->dbgmcu);

    /* RTC (alarm A/B: EXTI line 17 -> RTC_Alarm; wakeup -> RTC_WKUP), RNG */
    sysbus_create_varargs("stm32h7-rtc", 0x58004000,
                          qdev_get_gpio_in(armv7m, 41), qdev_get_gpio_in(armv7m, 3),
                          NULL);
    sysbus_create_simple("stm32h7-rng", 0x48021800, qdev_get_gpio_in(armv7m, 80));

    /* Independent watchdog of the M7 (IWDG2 belongs to the M4) */
    sysbus_create_simple("stm32h7-iwdg", 0x58004800, NULL);

    /* USB OTG HS (OTG1) and FS (OTG2), no cable */
    sysbus_create_simple("stm32h7-otg", 0x40040000, qdev_get_gpio_in(armv7m, 77));
    sysbus_create_simple("stm32h7-otg", 0x40080000, qdev_get_gpio_in(armv7m, 101));

    /* I2C1-4: event and error interrupts; bus "i2cN" */
    {
        static const hwaddr addr[4] = { 0x40005400, 0x40005800, 0x40005C00, 0x58001C00 };
        static const int irq[4][2] = { { 31, 32 }, { 33, 34 }, { 72, 73 }, { 95, 96 } };

        for (int i = 0; i < 4; i++) {
            DeviceState *d = qdev_new("stm32h7-i2c");
            g_autofree char *name = g_strdup_printf("i2c%d", i + 1);

            qdev_prop_set_string(d, "bus-name", name);
            object_property_add_child(OBJECT(s), name, OBJECT(d));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, addr[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(d), 0, qdev_get_gpio_in(armv7m, irq[i][0]));
            sysbus_connect_irq(SYS_BUS_DEVICE(d), 1, qdev_get_gpio_in(armv7m, irq[i][1]));
        }
    }

    /* Not modeled yet: accesses are logged with -d unimp */
    create_unimplemented_device("TIM2-7",      0x40000000, 0x1800);
    create_unimplemented_device("TIM12-14",    0x40001800, 0x0C00);
    create_unimplemented_device("LPTIM1",      0x40002400, 0x400);
    create_unimplemented_device("SPI2",        0x40003800, 0x400);
    create_unimplemented_device("SPI3",        0x40003C00, 0x400);
    create_unimplemented_device("SPDIFRX",     0x40004000, 0x400);
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
    create_unimplemented_device("ART",         0x40024400, 0x400);
    create_unimplemented_device("ETH",         0x40028000, 0x2400);
    create_unimplemented_device("DCMI",        0x48020000, 0x400);
    create_unimplemented_device("SDMMC2",      0x48022400, 0x400);
    create_unimplemented_device("RAMECC2",     0x48023000, 0x400);
    create_unimplemented_device("LTDC",        0x50001000, 0x1000);
    create_unimplemented_device("WWDG1",       0x50003000, 0x400);
    create_unimplemented_device("MDMA",        0x52000000, 0x1000);
    create_unimplemented_device("DMA2D",       0x52001000, 0x1000);
    create_unimplemented_device("JPEG",        0x52003000, 0x1000);
    create_unimplemented_device("FMC",         0x52004000, 0x1000);
    create_unimplemented_device("SDMMC1",      0x52007000, 0x1000);
    create_unimplemented_device("RAMECC1",     0x52009000, 0x1000);
    create_unimplemented_device("EXTI",        0x58000000, 0x400);
    create_unimplemented_device("SYSCFG",      0x58000400, 0x400);
    create_unimplemented_device("SPI6",        0x58001400, 0x400);
    create_unimplemented_device("LPTIM2-5",    0x58002400, 0x1000);
    create_unimplemented_device("COMP",        0x58003800, 0x400);
    create_unimplemented_device("VREFBUF",     0x58003C00, 0x400);
    create_unimplemented_device("IWDG2",       0x58004C00, 0x400);
    create_unimplemented_device("SAI4",        0x58005400, 0x400);
    create_unimplemented_device("CRC",         0x58024C00, 0x400);
    create_unimplemented_device("BDMA",        0x58025400, 0x400);
    create_unimplemented_device("DMAMUX2",     0x58025800, 0x400);
    create_unimplemented_device("RAMECC3",     0x58027000, 0x400);
}

static const Property stm32h745_soc_props[] = {
    DEFINE_PROP_UINT32("uid-seed", Stm32h745SocState, uid_seed, 0),
    DEFINE_PROP_STRING("adc-mv", Stm32h745SocState, adc_mv),
};

static void stm32h745_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32h745_soc_realize;
    device_class_set_props(dc, stm32h745_soc_props);
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
