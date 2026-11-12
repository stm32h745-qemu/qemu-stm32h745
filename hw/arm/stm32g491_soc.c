/*
 * STM32G491 SoC model (Cortex-M4F), the 512 KB part of the STM32G4 series.
 *
 * Memory map and interrupt numbers from RM0440 and the STM32G491 datasheet.
 * Modeled: 512 KB flash (dual bank, programmed through the flash interface,
 * aliased at 0), 112 KB SRAM (SRAM1+SRAM2+CCM, CCM also at 0x10000000),
 * system memory with the unique ID / flash size / calibration values, RCC
 * with the clock tree, PWR, flash interface, SYSCFG+EXTI, GPIOA-G with
 * externally drivable inputs, USART1-3, UART4-5 and LPUART1 (stm32h7-usart
 * model; serial_hd(0..5)), FDCAN1/2 (SLCAN chardevs "fdcan1"/"fdcan2"),
 * ADC1-3 (inputs in mV, timer triggers, DMA), DMA1/2 with DMAMUX, TIM1-4,
 * 6-8, 15-17 and 20 (counter core, update events, TRGO), IWDG (resets the
 * machine), RNG, I2C1-3 (buses "i2c1".."i2c3"), DBGMCU IDCODE. Everything
 * else (RTC, TAMP, WWDG, SPI, USB, DAC, comparators, ...) is an unimplemented
 * device that logs accesses (-d unimp).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "system/system.h"
#include "chardev/char.h"
#include "hw/arm/stm32g491_soc.h"
#include "hw/core/split-irq.h"
#include "hw/irq.h"
#include "hw/or-irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/misc/unimp.h"

#define FLASH_BASE    0x08000000
#define FLASH_SIZE    (512 * KiB)
#define SRAM_BASE     0x20000000
#define SRAM_SIZE     (112 * KiB)
#define CCM_BASE      0x10000000
#define CCM_SIZE      (16 * KiB)
#define SYSMEM_BASE   0x1FFF0000
#define SYSMEM_SIZE   (32 * KiB)
#define FDCAN_RAM_BASE 0x4000A400
#define FDCAN_MRAM_PER_INST 0x350

static const hwaddr usart_addr[STM32G491_NUM_USARTS] = {
    0x40013800, 0x40004400, 0x40004800, 0x40004C00, 0x40005000, 0x40008000,
};
static const int usart_irq[STM32G491_NUM_USARTS] = { 37, 38, 39, 52, 53, 91 };
static const bool usart_apb2[STM32G491_NUM_USARTS] = { true, false, false, false, false, false };
/* DMAMUX request IDs (RX; TX is RX + 1) */
static const int usart_dmareq[STM32G491_NUM_USARTS] = { 24, 26, 28, 30, 32, 34 };

static const hwaddr fdcan_addr[STM32G491_NUM_FDCANS] = { 0x40006400, 0x40006800 };
static const int fdcan_irq[STM32G491_NUM_FDCANS][2] = { { 21, 22 }, { 86, 87 } };

typedef struct TimDesc {
    const char *name;
    hwaddr addr;
    int irq;            /* update interrupt (NVIC) */
    bool apb2;
    bool wide;
    int adc_trig;       /* EXTSEL value of its TRGO on the ADCs, or -1 */
} TimDesc;

static const TimDesc tim_desc[STM32G491_NUM_TIMS] = {
    { "tim1",  0x40012C00, 25, true,  false, 9 },
    { "tim2",  0x40000000, 28, false, true,  11 },
    { "tim3",  0x40000400, 29, false, false, 4 },
    { "tim4",  0x40000800, 30, false, false, 12 },
    { "tim6",  0x40001000, 54, false, false, 13 },
    { "tim7",  0x40001400, 55, false, false, 22 },
    { "tim8",  0x40013400, 44, true,  false, 7 },
    { "tim15", 0x40014000, 24, true,  false, 14 },
    { "tim16", 0x40014400, 25, true,  false, -1 },
    { "tim17", 0x40014800, 26, true,  false, -1 },
};

static void stm32g491_soc_init(Object *obj)
{
    Stm32g491SocState *s = STM32G491_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "sysctrl", &s->sysctrl, TYPE_STM32G4_SYSCTRL);
    object_initialize_child(obj, "exti", &s->exti, TYPE_STM32G4_EXTI);
    for (int i = 0; i < STM32G491_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("gpio%c", 'a' + i);

        object_initialize_child(obj, name, &s->gpio[i], TYPE_STM32G4_GPIO);
    }
    for (int i = 0; i < STM32G491_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i], TYPE_STM32H7_USART);
    }
    for (int i = 0; i < STM32G491_NUM_FDCANS; i++) {
        g_autofree char *name = g_strdup_printf("fdcan%d", i + 1);

        object_initialize_child(obj, name, &s->fdcan[i], TYPE_STM32H7_FDCAN);
    }
    object_initialize_child(obj, "adc12", &s->adc12, TYPE_STM32G4_ADC);
    object_initialize_child(obj, "adc3", &s->adc3, TYPE_STM32G4_ADC);
    for (int i = 0; i < STM32G491_NUM_TIMS; i++) {
        object_initialize_child(obj, tim_desc[i].name, &s->tim[i], TYPE_STM32G4_TIM);
    }
    s->refclk = clock_new(obj, "refclk");
    s->pclk = clock_new(obj, "pclk");
}

/* DBGMCU: IDCODE = STM32G491/G4A1 (DEV_ID 0x479) */
static uint64_t dbgmcu_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g491SocState *s = opaque;

    switch (off) {
    case 0x00: return 0x20010479;
    case 0x04: return s->dbgmcu_cr;
    }
    return 0;
}

static void dbgmcu_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32g491SocState *s = opaque;

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
static bool apply_adc_inputs(Stm32g491SocState *s, Error **errp)
{
    g_auto(GStrv) items = NULL;

    if (!s->adc_mv || !*s->adc_mv) {
        return true;
    }
    items = g_strsplit_set(s->adc_mv, ",;", -1);
    for (char **it = items; *it; it++) {
        unsigned adc, ch;
        double mv;

        if (sscanf(*it, "%u.%u=%lf", &adc, &ch, &mv) != 3 || adc < 1 || adc > 3 ||
            ch >= STM32G4_ADC_CHANNELS) {
            error_setg(errp, "adc-mv: bad item '%s' (want <adc 1-3>.<ch 0-18>=<mV>)", *it);
            return false;
        }
        if (adc == 3) {
            stm32g4_adc_set_input_mv(&s->adc3, 0, ch, mv);
        } else {
            stm32g4_adc_set_input_mv(&s->adc12, adc - 1, ch, mv);
        }
    }
    return true;
}

/* gpio-in: ';' or ',' separated "<port><pin>=<0|1>", e.g. "C0=1;A8=1" */
static bool apply_gpio_inputs(Stm32g491SocState *s, Error **errp)
{
    g_auto(GStrv) items = NULL;

    if (!s->gpio_in || !*s->gpio_in) {
        return true;
    }
    items = g_strsplit_set(s->gpio_in, ",;", -1);
    for (char **it = items; *it; it++) {
        char port;
        unsigned pin, level;

        if (sscanf(*it, "%c%u=%u", &port, &pin, &level) != 3 || !g_ascii_isalpha(port) ||
            g_ascii_toupper(port) - 'A' >= STM32G491_NUM_GPIOS || pin > 15 || level > 1) {
            error_setg(errp, "gpio-in: bad item '%s' (want <port A-G><pin 0-15>=<0|1>)", *it);
            return false;
        }
        stm32g4_gpio_set_input(&s->gpio[g_ascii_toupper(port) - 'A'], pin, level);
    }
    return true;
}

/* Load Intel HEX records into the flash array; returns false on error */
static bool load_hex(const char *path, uint8_t *flash, Error **errp)
{
    g_autofree char *text = NULL;
    g_auto(GStrv) lines = NULL;
    uint32_t base = 0;
    gsize len;

    if (!g_file_get_contents(path, &text, &len, NULL)) {
        error_setg(errp, "flash-image: cannot read '%s'", path);
        return false;
    }
    lines = g_strsplit_set(text, "\r\n", -1);
    for (char **l = lines; *l; l++) {
        const char *p = *l;
        uint8_t rec[300];
        unsigned n, sum = 0;
        size_t plen = strlen(p);

        if (plen == 0) {
            continue;
        }
        if (p[0] != ':' || plen < 11 || (plen - 1) % 2) {
            error_setg(errp, "flash-image: '%s': bad record '%s'", path, p);
            return false;
        }
        n = (plen - 1) / 2;
        if (n > sizeof(rec)) {
            error_setg(errp, "flash-image: '%s': record too long", path);
            return false;
        }
        for (unsigned i = 0; i < n; i++) {
            unsigned v;

            if (sscanf(p + 1 + 2 * i, "%2x", &v) != 1) {
                error_setg(errp, "flash-image: '%s': bad hex digits", path);
                return false;
            }
            rec[i] = v;
            sum += v;
        }
        if ((sum & 0xFF) != 0 || rec[0] + 5u != n) {
            error_setg(errp, "flash-image: '%s': bad checksum or length", path);
            return false;
        }
        switch (rec[3]) {
        case 0: {
            uint32_t addr = base + ((rec[1] << 8) | rec[2]);

            if (addr < FLASH_BASE || addr + rec[0] > FLASH_BASE + FLASH_SIZE) {
                error_setg(errp, "flash-image: '%s': address 0x%08x outside the flash",
                           path, addr);
                return false;
            }
            memcpy(flash + (addr - FLASH_BASE), rec + 4, rec[0]);
            break;
        }
        case 1:
            return true;
        case 2:
            base = ((rec[4] << 8) | rec[5]) << 4;
            break;
        case 4:
            base = ((rec[4] << 8) | rec[5]) << 16;
            break;
        default:
            break;
        }
    }
    return true;
}

static bool load_flash_image(Stm32g491SocState *s, Error **errp)
{
    g_auto(GStrv) items = NULL;
    uint8_t *flash = stm32g4_sysctrl_flash_ptr(&s->sysctrl);

    if (!s->flash_image || !*s->flash_image) {
        return true;
    }
    items = g_strsplit(s->flash_image, ":", -1);
    for (char **it = items; *it; it++) {
        char *at = strrchr(*it, '@');

        if (g_str_has_suffix(*it, ".hex") || g_str_has_suffix(*it, ".ihex")) {
            if (!load_hex(*it, flash, errp)) {
                return false;
            }
        } else if (at) {
            g_autofree char *file = g_strndup(*it, at - *it);
            g_autofree char *data = NULL;
            gsize len;
            uint64_t off = g_ascii_strtoull(at + 1, NULL, 0);

            if (!g_file_get_contents(file, &data, &len, NULL) || off + len > FLASH_SIZE) {
                error_setg(errp, "flash-image: cannot load '%s' at 0x%" PRIx64, file, off);
                return false;
            }
            memcpy(flash + off, data, len);
        } else {
            error_setg(errp, "flash-image: '%s': want file.hex or file.bin@offset", *it);
            return false;
        }
    }
    return true;
}

typedef struct KernelClk {
    Stm32g491SocState *soc;
    int which;
} KernelClk;

static uint64_t kclk_hz(void *opaque)
{
    KernelClk *k = opaque;

    return stm32g4_sysctrl_hz(&k->soc->sysctrl, k->which);
}

static uint64_t adc12_kernel_hz(void *opaque)
{
    return stm32g4_sysctrl_hz(&((Stm32g491SocState *)opaque)->sysctrl, STM32G4_CLK_ADC12);
}

static uint64_t adc3_kernel_hz(void *opaque)
{
    return stm32g4_sysctrl_hz(&((Stm32g491SocState *)opaque)->sysctrl, STM32G4_CLK_ADC345);
}

static uint64_t hclk_hz(void *opaque)
{
    return stm32g4_sysctrl_hz(&((Stm32g491SocState *)opaque)->sysctrl, STM32G4_CLK_HCLK);
}

static void stm32g491_soc_realize(DeviceState *dev, Error **errp)
{
    Stm32g491SocState *s = STM32G491_SOC(dev);
    MemoryRegion *sysmem = get_system_memory();
    DeviceState *armv7m, *nvic_dev;
    SysBusDevice *busdev;
    DeviceState *ormux;

    /* RCC, PWR, flash (array + interface); the core clock comes from here */
    qdev_prop_set_uint32(DEVICE(&s->sysctrl), "hse-hz", s->hse_hz);
    busdev = SYS_BUS_DEVICE(&s->sysctrl);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    sysbus_mmio_map(busdev, 0, 0x40021000);        /* RCC */
    sysbus_mmio_map(busdev, 1, 0x40007000);        /* PWR */
    sysbus_mmio_map(busdev, 2, 0x40022000);        /* flash interface */
    sysbus_mmio_map(busdev, 3, FLASH_BASE);        /* flash array */
    memory_region_init_alias(&s->flash_alias, OBJECT(s), "stm32g491.flash-alias",
                             sysbus_mmio_get_region(busdev, 3), 0, FLASH_SIZE);
    memory_region_add_subregion(sysmem, 0, &s->flash_alias);

    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysctrl.hclk);

    /* SRAM1 + SRAM2 + CCM SRAM (contiguous), CCM also at 0x10000000. The 1 KB
     * retained region the firmware keeps at the top is ordinary RAM: a system
     * reset does not touch it and a fresh start has it zeroed. */
    memory_region_init_ram(&s->sram, OBJECT(s), "stm32g491.sram", SRAM_SIZE, errp);
    memory_region_add_subregion(sysmem, SRAM_BASE, &s->sram);
    memory_region_init_alias(&s->ccm_alias, OBJECT(s), "stm32g491.ccm", &s->sram,
                             SRAM_SIZE - CCM_SIZE, CCM_SIZE);
    memory_region_add_subregion(sysmem, CCM_BASE, &s->ccm_alias);
    memory_region_init_ram(&s->fdcan_ram, OBJECT(s), "stm32g491.sramcan",
                           STM32G491_NUM_FDCANS * FDCAN_MRAM_PER_INST, errp);
    memory_region_add_subregion(sysmem, FDCAN_RAM_BASE, &s->fdcan_ram);

    /* System memory (read only): OTP area, package data, unique ID, flash
     * size and the factory calibration values the HAL reads */
    if (!memory_region_init_rom(&s->sysmem, OBJECT(s), "stm32g491.sysmem", SYSMEM_SIZE,
                                errp)) {
        return;
    }
    {
        uint8_t *p = memory_region_get_ram_ptr(&s->sysmem);
        uint32_t uid[3] = { 0x00410022 ^ s->uid_seed, 0x3133510D + s->uid_seed,
                            0x30393538 };

        memset(p, 0xFF, SYSMEM_SIZE);
        stl_le_p(p + 0x7500, 0xFFFFFF00);                  /* package data */
        for (int i = 0; i < 3; i++) {
            stl_le_p(p + 0x7590 + 4 * i, uid[i]);
        }
        stw_le_p(p + 0x75E0, FLASH_SIZE / KiB);
        stw_le_p(p + 0x75A8, 1040);                        /* TS_CAL1 (30 C) */
        stw_le_p(p + 0x75AA, 1656);                        /* VREFINT_CAL */
        stw_le_p(p + 0x75CA, 1370);                        /* TS_CAL2 (130 C) */
    }
    memory_region_add_subregion(sysmem, SYSMEM_BASE, &s->sysmem);

    if (!load_flash_image(s, errp)) {
        return;
    }

    /* Cortex-M4F, vector table at the start of the flash */
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 102);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(armv7m, "init-nsvtor", FLASH_BASE);
    qdev_prop_set_uint32(armv7m, "mpu-ns-regions", 8);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysctrl.hclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(sysmem), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }
    nvic_dev = armv7m;
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(nvic_dev, 4));      /* FLASH */

    /* DMA1, DMA2 and DMAMUX1 (outputs 0-7: DMA1 channels, 8-15: DMA2) */
    {
        static const int dma_irq[2][8] = {
            { 11, 12, 13, 14, 15, 16, 17, 96 }, { 56, 57, 58, 59, 60, 97, 98, 99 },
        };

        for (int d = 0; d < 2; d++) {
            s->dma[d] = qdev_new("stm32g4-dma");
            object_property_add_child(OBJECT(s), d ? "dma2" : "dma1", OBJECT(s->dma[d]));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(s->dma[d]), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(s->dma[d]), 0, 0x40020000 + 0x400 * d);
            for (int n = 0; n < 8; n++) {
                sysbus_connect_irq(SYS_BUS_DEVICE(s->dma[d]), n,
                                   qdev_get_gpio_in(nvic_dev, dma_irq[d][n]));
            }
        }
        s->dmamux = qdev_new("stm32g4-dmamux");
        object_property_add_child(OBJECT(s), "dmamux1", OBJECT(s->dmamux));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->dmamux), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->dmamux), 0, 0x40020800);
        for (int c = 0; c < 16; c++) {
            qdev_connect_gpio_out_named(s->dmamux, "out", c,
                                        qdev_get_gpio_in_named(s->dma[c / 8], "req", c % 8));
        }
    }

    /* GPIOA..G and EXTI/SYSCFG */
    busdev = SYS_BUS_DEVICE(&s->exti);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    sysbus_mmio_map(busdev, 0, 0x40010400);
    sysbus_mmio_map(busdev, 1, 0x40010000);
    {
        static const int exti_irq[STM32G4_EXTI_IRQS] = { 6, 7, 8, 9, 10, 23, 40 };

        for (int i = 0; i < STM32G4_EXTI_IRQS; i++) {
            sysbus_connect_irq(busdev, i, qdev_get_gpio_in(nvic_dev, exti_irq[i]));
        }
    }
    for (int i = 0; i < STM32G491_NUM_GPIOS; i++) {
        DeviceState *g = DEVICE(&s->gpio[i]);
        g_autofree char *name = g_strdup_printf("%c", 'A' + i);

        qdev_prop_set_string(g, "name", name);
        qdev_prop_set_uint32(g, "mode-reset", i == 0 ? 0xABFFFFFF : i == 1 ? 0xFFFFFEBF : 0xFFFFFFFF);
        qdev_prop_set_uint32(g, "ospeed-reset", i == 0 ? 0x0C000000 : 0);
        qdev_prop_set_uint32(g, "pupd-reset", i == 0 ? 0x64000000 : i == 1 ? 0x00000100 : 0);
        busdev = SYS_BUS_DEVICE(g);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, 0x48000000 + i * 0x400);
        for (int pin = 0; pin < 16; pin++) {
            qdev_connect_gpio_out_named(g, "pad", pin,
                                        qdev_get_gpio_in_named(DEVICE(&s->exti), "gpio",
                                                               i * 16 + pin));
        }
    }

    if (!apply_gpio_inputs(s, errp)) {
        return;
    }

    /* USARTs: serial_hd(0..5) = USART1, USART2, USART3, UART4, UART5, LPUART1 */
    for (int i = 0; i < STM32G491_NUM_USARTS; i++) {
        DeviceState *u = DEVICE(&s->usart[i]);
        KernelClk *clk = g_new(KernelClk, 1);

        clk->soc = s;
        clk->which = usart_apb2[i] ? STM32G4_CLK_PCLK2 : STM32G4_CLK_PCLK1;
        s->usart[i].kernel_hz = kclk_hz;
        s->usart[i].kernel_hz_opaque = clk;
        qdev_prop_set_chr(u, "chardev", serial_hd(i));
        qdev_connect_clock_in(u, "clk", s->pclk);
        busdev = SYS_BUS_DEVICE(u);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(nvic_dev, usart_irq[i]));
        qdev_connect_gpio_out_named(u, "dma-rx", 0,
                                    qdev_get_gpio_in_named(s->dmamux, "req", usart_dmareq[i]));
        qdev_connect_gpio_out_named(u, "dma-tx", 0,
                                    qdev_get_gpio_in_named(s->dmamux, "req",
                                                           usart_dmareq[i] + 1));
    }
    clock_set_hz(s->pclk, 16000000);

    /* FDCAN1/2: bus = chardev "fdcan1"/"fdcan2" (SLCAN), if given */
    for (int i = 0; i < STM32G491_NUM_FDCANS; i++) {
        DeviceState *f = DEVICE(&s->fdcan[i]);
        g_autofree char *id = g_strdup_printf("fdcan%d", i + 1);
        Chardev *chr = qemu_chr_find(id);

        if (chr) {
            qdev_prop_set_chr(f, "chardev", chr);
        }
        qdev_prop_set_bit(f, "g4", true);
        qdev_prop_set_uint32(f, "msg-ram-offset", i * FDCAN_MRAM_PER_INST);
        object_property_set_link(OBJECT(f), "msg-ram", OBJECT(&s->fdcan_ram), &error_abort);
        busdev = SYS_BUS_DEVICE(f);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, fdcan_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(nvic_dev, fdcan_irq[i][0]));
        sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(nvic_dev, fdcan_irq[i][1]));
    }

    /* ADC1+ADC2 (shared interrupt 18) and ADC3 (47) */
    qdev_prop_set_uint32(DEVICE(&s->adc12), "num", 2);
    qdev_prop_set_uint32(DEVICE(&s->adc12), "first", 1);
    qdev_prop_set_uint32(DEVICE(&s->adc3), "num", 1);
    qdev_prop_set_uint32(DEVICE(&s->adc3), "first", 3);
    qdev_prop_set_uint32(DEVICE(&s->adc12), "vref-mv", s->vref_mv);
    qdev_prop_set_uint32(DEVICE(&s->adc3), "vref-mv", s->vref_mv);
    s->adc12.kernel_hz = adc12_kernel_hz;
    s->adc3.kernel_hz = adc3_kernel_hz;
    s->adc12.hclk_hz = s->adc3.hclk_hz = hclk_hz;
    s->adc12.clk_opaque = s->adc3.clk_opaque = s;
    if (!apply_adc_inputs(s, errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(&s->adc12), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(&s->adc3), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adc12), 0, 0x50000000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc12), 0, qdev_get_gpio_in(nvic_dev, 18));
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adc3), 0, 0x50000400);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc3), 0, qdev_get_gpio_in(nvic_dev, 47));
    qdev_connect_gpio_out_named(DEVICE(&s->adc12), "dma-req0", 0,
                                qdev_get_gpio_in_named(s->dmamux, "req", 5));
    qdev_connect_gpio_out_named(DEVICE(&s->adc12), "dma-req1", 0,
                                qdev_get_gpio_in_named(s->dmamux, "req", 36));
    qdev_connect_gpio_out_named(DEVICE(&s->adc3), "dma-req0", 0,
                                qdev_get_gpio_in_named(s->dmamux, "req", 37));

    /* Timers; update interrupt on the NVIC, TRGO to the ADC external triggers */
    ormux = qdev_new(TYPE_OR_IRQ);
    object_property_add_child(OBJECT(s), "tim-irq25", OBJECT(ormux));
    qdev_prop_set_uint16(ormux, "num-lines", 2);
    qdev_realize_and_unref(ormux, NULL, &error_fatal);
    qdev_connect_gpio_out(ormux, 0, qdev_get_gpio_in(nvic_dev, 25));
    for (int i = 0; i < STM32G491_NUM_TIMS; i++) {
        DeviceState *t = DEVICE(&s->tim[i]);
        KernelClk *clk = g_new(KernelClk, 1);

        clk->soc = s;
        clk->which = tim_desc[i].apb2 ? STM32G4_CLK_TIMCLK2 : STM32G4_CLK_TIMCLK1;
        s->tim[i].clk_hz = kclk_hz;
        s->tim[i].clk_opaque = clk;
        qdev_prop_set_uint32(t, "width", tim_desc[i].wide ? 32 : 16);
        busdev = SYS_BUS_DEVICE(t);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, tim_desc[i].addr);
        if (tim_desc[i].irq == 25) {
            sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(ormux, i == 0 ? 0 : 1));
        } else {
            sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(nvic_dev, tim_desc[i].irq));
        }
        if (tim_desc[i].adc_trig >= 0) {
            DeviceState *sp = qdev_new(TYPE_SPLIT_IRQ);

            object_property_add_child(OBJECT(s), "trgo-split[*]", OBJECT(sp));
            qdev_prop_set_uint16(sp, "num-lines", 2);
            qdev_realize_and_unref(sp, NULL, &error_fatal);
            qdev_connect_gpio_out_named(t, "trgo", 0, qdev_get_gpio_in(sp, 0));
            qdev_connect_gpio_out(sp, 0, qdev_get_gpio_in_named(DEVICE(&s->adc12), "trig",
                                                               tim_desc[i].adc_trig));
            qdev_connect_gpio_out(sp, 1, qdev_get_gpio_in_named(DEVICE(&s->adc3), "trig",
                                                               tim_desc[i].adc_trig));
        }
    }

    memory_region_init_io(&s->dbgmcu, OBJECT(s), &dbgmcu_ops, s, "stm32g491.dbgmcu", 0x400);
    /* on the external PPB: overlay it on the ARMv7-M container */
    memory_region_add_subregion_overlap(&s->armv7m.container, 0xE0042000, &s->dbgmcu, 10);

    /* Independent watchdog (resets the machine on timeout), RNG */
    sysbus_create_simple("stm32h7-iwdg", 0x40003000, NULL);
    sysbus_create_simple("stm32h7-rng", 0x50060800, qdev_get_gpio_in(nvic_dev, 90));

    /* I2C1-3: event and error interrupts; bus "i2cN" */
    {
        static const hwaddr addr[3] = { 0x40005400, 0x40005800, 0x40007800 };
        static const int irq[3][2] = { { 31, 32 }, { 33, 34 }, { 92, 93 } };

        for (int i = 0; i < 3; i++) {
            DeviceState *d = qdev_new("stm32h7-i2c");
            g_autofree char *name = g_strdup_printf("i2c%d", i + 1);

            qdev_prop_set_string(d, "bus-name", name);
            object_property_add_child(OBJECT(s), name, OBJECT(d));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, addr[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(d), 0, qdev_get_gpio_in(nvic_dev, irq[i][0]));
            sysbus_connect_irq(SYS_BUS_DEVICE(d), 1, qdev_get_gpio_in(nvic_dev, irq[i][1]));
        }
    }

    /* Not modeled: accesses are logged with -d unimp */
    create_unimplemented_device("TAMP",     0x40002400, 0x400);
    create_unimplemented_device("RTC",      0x40002800, 0x400);
    create_unimplemented_device("WWDG",     0x40002C00, 0x400);
    create_unimplemented_device("SPI2",     0x40003800, 0x400);
    create_unimplemented_device("SPI3",     0x40003C00, 0x400);
    create_unimplemented_device("USB",      0x40005C00, 0x400);
    create_unimplemented_device("USB-SRAM", 0x40006000, 0x400);
    create_unimplemented_device("LPTIM1",   0x40007C00, 0x400);
    create_unimplemented_device("UCPD1",    0x4000A000, 0x400);
    create_unimplemented_device("VREFBUF",  0x40010030, 0x20);
    create_unimplemented_device("COMP",     0x40010200, 0x100);
    create_unimplemented_device("OPAMP",    0x40010300, 0x100);
    create_unimplemented_device("TIM20",    0x40015000, 0x400);
    create_unimplemented_device("SPI1",     0x40013000, 0x400);
    create_unimplemented_device("SPI4",     0x40013C00, 0x400);
    create_unimplemented_device("SAI1",     0x40015400, 0x400);
    create_unimplemented_device("HRTIM",    0x40016800, 0x1000);
    create_unimplemented_device("CORDIC",   0x40020C00, 0x400);
    create_unimplemented_device("FMAC",     0x40021400, 0x400);
    create_unimplemented_device("CRC",      0x40023000, 0x400);
    create_unimplemented_device("DAC1-4",   0x50000800, 0x1000);
    create_unimplemented_device("QUADSPI",  0xA0001000, 0x400);
}

static const Property stm32g491_soc_props[] = {
    DEFINE_PROP_UINT32("uid-seed", Stm32g491SocState, uid_seed, 0),
    DEFINE_PROP_UINT32("hse-hz", Stm32g491SocState, hse_hz, 8000000),
    DEFINE_PROP_UINT32("vref-mv", Stm32g491SocState, vref_mv, 3300),
    DEFINE_PROP_STRING("adc-mv", Stm32g491SocState, adc_mv),
    DEFINE_PROP_STRING("gpio-in", Stm32g491SocState, gpio_in),
    DEFINE_PROP_STRING("flash-image", Stm32g491SocState, flash_image),
};

static void stm32g491_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32g491_soc_realize;
    device_class_set_props(dc, stm32g491_soc_props);
    dc->user_creatable = false;
}

static const TypeInfo stm32g491_soc_info = {
    .name = TYPE_STM32G491_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g491SocState),
    .instance_init = stm32g491_soc_init,
    .class_init = stm32g491_soc_class_init,
};

static void stm32g491_soc_register(void)
{
    type_register_static(&stm32g491_soc_info);
}
type_init(stm32g491_soc_register)
