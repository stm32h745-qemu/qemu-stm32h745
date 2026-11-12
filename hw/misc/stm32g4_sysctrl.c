/*
 * STM32G4 system control: RCC, PWR, flash array and flash interface.
 * Register layouts from RM0440 (STM32G4 series).
 *
 * RCC: registers latch; every ready bit firmware polls follows its enable,
 * the clock switch follows SW once the source is on, and SYSCLK/HCLK/PCLKx
 * are computed from the HSI16/HSE/PLL settings so peripheral timing (SysTick,
 * timers, ADC conversion time, baud rates) follows what the firmware
 * configures. Reset flags in RCC_CSR (software reset, IWDG, power-on).
 *
 * Flash: the array is a ROM device. Writes are only accepted with CR.PG set
 * after unlocking, 64 bits at a time, and only into erased double words (or
 * writing zero), as on the part; page and mass erase work per bank. Content
 * survives a system reset.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/misc/stm32g4_sysctrl.h"
#include "hw/misc/stm32h7_sysctrl.h"
#include "migration/vmstate.h"
#include "system/runstate.h"
#include "exec/memory.h"

/* ---- RCC ---- */
#define RCC_CR       0x00
#define RCC_ICSCR    0x04
#define RCC_CFGR     0x08
#define RCC_PLLCFGR  0x0C
#define RCC_CIER     0x18
#define RCC_CIFR     0x1C
#define RCC_CICR     0x20
#define RCC_CCIPR    0x88
#define RCC_BDCR     0x90
#define RCC_CSR      0x94
#define RCC_CRRCR    0x98

#define CR_HSION     BIT(8)
#define CR_HSIRDY    BIT(10)
#define CR_HSEON     BIT(16)
#define CR_HSERDY    BIT(17)
#define CR_PLLON     BIT(24)
#define CR_PLLRDY    BIT(25)

#define CSR_LSION    BIT(0)
#define CSR_LSIRDY   BIT(1)
#define CSR_RMVF     BIT(23)
#define CSR_OBLRSTF  BIT(25)
#define CSR_PINRSTF  BIT(26)
#define CSR_BORRSTF  BIT(27)
#define CSR_SFTRSTF  BIT(28)
#define CSR_IWDGRSTF BIT(29)
#define CSR_FLAGS    0xFE800000u

#define HSI16_HZ     16000000

/* ---- PWR ---- */
#define PWR_CR1      0x00
#define PWR_SR2      0x14
#define PWR_SCR      0x18
#define PWR_CR5      0x80

/* ---- Flash interface ---- */
#define FL_ACR       0x00
#define FL_PDKEYR    0x04
#define FL_KEYR      0x08
#define FL_OPTKEYR   0x0C
#define FL_SR        0x10
#define FL_CR        0x14
#define FL_ECCR      0x18
#define FL_OPTR      0x20

#define SR_EOP       BIT(0)
#define SR_OPERR     BIT(1)
#define SR_PROGERR   BIT(3)
#define SR_WRPERR    BIT(4)
#define SR_PGAERR    BIT(5)
#define SR_SIZERR    BIT(6)
#define SR_PGSERR    BIT(7)
#define SR_MISERR    BIT(8)
#define SR_FASTERR   BIT(9)
#define SR_RDERR     BIT(14)
#define SR_OPTVERR   BIT(15)
#define SR_ERRORS    (SR_OPERR | SR_PROGERR | SR_WRPERR | SR_PGAERR | SR_SIZERR | \
                      SR_PGSERR | SR_MISERR | SR_FASTERR | SR_RDERR | SR_OPTVERR)
#define SR_W1C       (SR_EOP | SR_ERRORS)

#define CR_PG        BIT(0)
#define CR_PER       BIT(1)
#define CR_MER1      BIT(2)
#define CR_PNB_SHIFT 3
#define CR_PNB_MASK  0xFF
#define CR_BKER      BIT(11)
#define CR_MER2      BIT(15)
#define CR_STRT      BIT(16)
#define CR_OPTSTRT   BIT(17)
#define CR_FSTPG     BIT(18)
#define CR_EOPIE     BIT(24)
#define CR_ERRIE     BIT(25)
#define CR_OBL_LAUNCH BIT(27)
#define CR_OPTLOCK   BIT(30)
#define CR_LOCK      BIT(31)

#define OPTR_DBANK   BIT(22)

#define FLASH_KEY1   0x45670123
#define FLASH_KEY2   0xCDEF89AB
#define OPT_KEY1     0x08192A3B
#define OPT_KEY2     0x4C5D6E7F

#define FL(s, off)   ((s)->fl[(off) / 4])
#define RCC(s, off)  ((s)->rcc[(off) / 4])

/* ---- clock tree ---- */

static uint64_t src_hz(Stm32g4SysctrlState *s, uint32_t sel)
{
    switch (sel) {
    case 1: return (RCC(s, RCC_CR) & CR_HSION) ? HSI16_HZ : 0;
    case 2: return (RCC(s, RCC_CR) & CR_HSEON) ? s->hse_hz : 0;
    }
    return 0;
}

static uint64_t pll_vco_hz(Stm32g4SysctrlState *s)
{
    uint32_t c = RCC(s, RCC_PLLCFGR);
    /* PLLSRC: 2 HSI16, 3 HSE (SW/SWS encoding: 1 HSI16, 2 HSE) */
    uint64_t in = src_hz(s, (c & 3) == 2 ? 1 : (c & 3) == 3 ? 2 : 0);
    uint32_t m = ((c >> 4) & 0xF) + 1, n = (c >> 8) & 0x7F;

    if ((c & 3) < 2 || n < 8) {
        return 0;
    }
    return in / m * n;
}

static uint64_t pll_r_hz(Stm32g4SysctrlState *s)
{
    return pll_vco_hz(s) / (2 * (((RCC(s, RCC_PLLCFGR) >> 25) & 3) + 1));
}

static uint64_t pll_p_hz(Stm32g4SysctrlState *s)
{
    uint32_t c = RCC(s, RCC_PLLCFGR);
    uint32_t div = (c >> 27) & 0x1F;

    if (!div) {
        div = (c & BIT(17)) ? 17 : 7;
    }
    return pll_vco_hz(s) / div;
}

static uint64_t sysclk_hz(Stm32g4SysctrlState *s)
{
    switch (s->sws) {
    case 1: return HSI16_HZ;
    case 2: return s->hse_hz;
    case 3: return pll_r_hz(s);
    }
    return HSI16_HZ;
}

static uint64_t hclk_hz(Stm32g4SysctrlState *s)
{
    static const unsigned div[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
    uint32_t hpre = (RCC(s, RCC_CFGR) >> 4) & 0xF;

    return (hpre & 8) ? sysclk_hz(s) / div[hpre & 7] : sysclk_hz(s);
}

static uint64_t pclk_hz(Stm32g4SysctrlState *s, int shift, bool timer)
{
    uint32_t ppre = (RCC(s, RCC_CFGR) >> shift) & 7;
    uint64_t h = hclk_hz(s);

    if (ppre & 4) {
        h >>= (ppre & 3) + 1;
        return timer ? h * 2 : h;
    }
    return h;
}

uint64_t stm32g4_sysctrl_hz(Stm32g4SysctrlState *s, Stm32g4Clk clk)
{
    uint32_t ccipr = RCC(s, RCC_CCIPR);

    switch (clk) {
    case STM32G4_CLK_SYSCLK:  return sysclk_hz(s);
    case STM32G4_CLK_HCLK:    return hclk_hz(s);
    case STM32G4_CLK_PCLK1:   return pclk_hz(s, 8, false);
    case STM32G4_CLK_PCLK2:   return pclk_hz(s, 11, false);
    case STM32G4_CLK_TIMCLK1: return pclk_hz(s, 8, true);
    case STM32G4_CLK_TIMCLK2: return pclk_hz(s, 11, true);
    case STM32G4_CLK_ADC12:
    case STM32G4_CLK_ADC345: {
        uint32_t sel = (ccipr >> (clk == STM32G4_CLK_ADC12 ? 28 : 30)) & 3;

        return sel == 1 ? pll_p_hz(s) : sel == 2 ? sysclk_hz(s) : 0;
    }
    }
    return 0;
}

static void update_clocks(Stm32g4SysctrlState *s)
{
    uint64_t hz = hclk_hz(s);

    clock_update_hz(s->hclk, hz ? hz : HSI16_HZ);
}

/* ---- RCC registers ---- */

static uint64_t rcc_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;
    uint32_t v = s->rcc[off / 4];

    switch (off) {
    case RCC_CR:
        v &= ~(CR_HSIRDY | CR_HSERDY | CR_PLLRDY);
        v |= (v & CR_HSION) ? CR_HSIRDY : 0;
        v |= (v & CR_HSEON) ? CR_HSERDY : 0;
        v |= (v & CR_PLLON) ? CR_PLLRDY : 0;
        break;
    case RCC_CFGR:
        v = (v & ~(3u << 2)) | (s->sws << 2);
        break;
    case RCC_BDCR:                  /* LSERDY follows LSEON */
        v = (v & ~BIT(1)) | ((v & BIT(0)) << 1);
        break;
    case RCC_CSR:
        v = (v & ~CSR_LSIRDY) | ((v & CSR_LSION) ? CSR_LSIRDY : 0);
        break;
    case RCC_CRRCR:                 /* HSI48RDY follows HSI48ON */
        v = (v & ~BIT(1)) | ((v & BIT(0)) << 1);
        break;
    }
    return v;
}

static void rcc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;

    switch (off) {
    case RCC_CIFR:
        return;
    case RCC_CICR:
        return;
    case RCC_CSR:
        if (val & CSR_RMVF) {
            s->rcc[off / 4] &= ~CSR_FLAGS;
        }
        s->rcc[off / 4] = (s->rcc[off / 4] & CSR_FLAGS) | (val & ~CSR_FLAGS & ~CSR_RMVF);
        return;
    case RCC_CR:
        /* PLL and HSE stay on while selected as the system clock */
        if (s->sws == 3) {
            val |= CR_PLLON;
        }
        if (s->sws == 2) {
            val |= CR_HSEON;
        }
        if (s->sws == 1) {
            val |= CR_HSION;
        }
        break;
    }
    s->rcc[off / 4] = val;
    if (off == RCC_CR || off == RCC_CFGR || off == RCC_PLLCFGR) {
        if (off == RCC_CFGR || off == RCC_CR) {
            uint32_t sw = RCC(s, RCC_CFGR) & 3;
            uint32_t cr = RCC(s, RCC_CR);
            bool ready = (sw == 0 || sw == 1) ? (cr & CR_HSION) :
                         sw == 2 ? (cr & CR_HSEON) : (cr & CR_PLLON);

            if (ready) {
                s->sws = sw == 0 ? 1 : sw;
            }
        }
        update_clocks(s);
    }
}

/* ---- PWR ---- */

static uint64_t pwr_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;

    if (off == PWR_SR2) {
        return 0;                   /* VOSF clear (scaling done), no PVD flags */
    }
    return off < sizeof(s->pwr) ? s->pwr[off / 4] : 0;
}

static void pwr_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;

    if (off == PWR_SCR || off >= sizeof(s->pwr)) {
        return;
    }
    s->pwr[off / 4] = val;
}

/* ---- Flash ---- */

static void fl_update_irq(Stm32g4SysctrlState *s)
{
    uint32_t cr = FL(s, FL_CR), sr = FL(s, FL_SR);

    qemu_set_irq(s->flash_irq, ((cr & CR_EOPIE) && (sr & SR_EOP)) ||
                               ((cr & CR_ERRIE) && (sr & SR_ERRORS)));
}

static void fl_dirty(Stm32g4SysctrlState *s, uint32_t start, uint32_t len)
{
    memory_region_set_dirty(&s->flash_mem, start, len);
}

static void flash_erase(Stm32g4SysctrlState *s, uint32_t cr)
{
    uint8_t *mem = memory_region_get_ram_ptr(&s->flash_mem);
    bool dbank = s->optr & OPTR_DBANK;
    uint32_t page_size = dbank ? 2048 : 4096;
    uint32_t bank_size = s->flash_size / 2;
    uint32_t start, len;

    if (cr & (CR_MER1 | CR_MER2)) {
        /* Mass erase of bank 1 and/or 2 (the whole array in single bank mode) */
        if (!dbank) {
            start = 0;
            len = s->flash_size;
        } else if ((cr & (CR_MER1 | CR_MER2)) == (CR_MER1 | CR_MER2)) {
            start = 0;
            len = s->flash_size;
        } else {
            start = (cr & CR_MER2) ? bank_size : 0;
            len = bank_size;
        }
    } else {
        uint32_t pnb = (cr >> CR_PNB_SHIFT) & CR_PNB_MASK;

        start = pnb * page_size;
        if (dbank && (cr & CR_BKER)) {
            start += bank_size;
        }
        len = page_size;
    }
    if (start + len > s->flash_size) {
        FL(s, FL_SR) |= SR_PGSERR;
        return;
    }
    memset(mem + start, 0xFF, len);
    fl_dirty(s, start, len);
    FL(s, FL_SR) |= SR_EOP;
}

static void flash_commit_dword(Stm32g4SysctrlState *s)
{
    uint8_t *mem = memory_region_get_ram_ptr(&s->flash_mem);
    uint32_t o = s->pg_addr;
    uint32_t old0 = ldl_le_p(mem + o), old1 = ldl_le_p(mem + o + 4);

    if ((old0 != 0xFFFFFFFF || old1 != 0xFFFFFFFF) &&
        (s->pg_word[0] || s->pg_word[1])) {
        FL(s, FL_SR) |= SR_PROGERR;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "stm32g4 flash: programming non-erased double word at 0x%x\n", o);
    } else {
        stl_le_p(mem + o, s->pg_word[0]);
        stl_le_p(mem + o + 4, s->pg_word[1]);
        fl_dirty(s, o, 8);
        FL(s, FL_SR) |= SR_EOP;
    }
    s->pg_pending = false;
    s->pg_have = 0;
}

static void flash_array_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;
    uint32_t cr = FL(s, FL_CR);

    if ((cr & CR_LOCK) || !(cr & CR_PG)) {
        FL(s, FL_SR) |= SR_PGSERR;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "stm32g4 flash: write to 0x%08x without PG/unlock\n",
                      (unsigned)addr + 0x08000000);
        fl_update_irq(s);
        return;
    }
    if (size == 8) {
        s->pg_addr = addr & ~7;
        stq_le_p(s->pg_word, data);
        s->pg_have = 3;
        flash_commit_dword(s);
        fl_update_irq(s);
        return;
    }
    if (size != 4) {
        FL(s, FL_SR) |= SR_SIZERR;
        fl_update_irq(s);
        return;
    }
    if (s->pg_pending && (addr & ~7) != s->pg_addr) {
        FL(s, FL_SR) |= SR_PGSERR;      /* first half never completed */
        s->pg_pending = false;
        s->pg_have = 0;
    }
    s->pg_pending = true;
    s->pg_addr = addr & ~7;
    s->pg_word[(addr >> 2) & 1] = data;
    s->pg_have |= 1 << ((addr >> 2) & 1);
    if (s->pg_have == 3) {
        flash_commit_dword(s);
    }
    fl_update_irq(s);
}

static const MemoryRegionOps flash_array_ops = {
    .write = flash_array_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t flash_if_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;

    switch (off) {
    case FL_OPTR:
        return s->optr;
    case FL_KEYR: case FL_OPTKEYR: case FL_PDKEYR:
        return 0;
    }
    return off < sizeof(s->fl) ? s->fl[off / 4] : 0;
}

static void flash_if_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32g4SysctrlState *s = opaque;

    switch (off) {
    case FL_ACR:
        /* LATENCY reads back; cache reset bits are self clearing */
        FL(s, FL_ACR) = val & ~(BIT(11) | BIT(12));
        return;
    case FL_KEYR:
        if (s->key_state == 0 && val == FLASH_KEY1) {
            s->key_state = 1;
        } else if (s->key_state == 1 && val == FLASH_KEY2) {
            FL(s, FL_CR) &= ~CR_LOCK;
            s->key_state = 0;
        } else {
            s->key_state = 0;
        }
        return;
    case FL_OPTKEYR:
        if (s->optkey_state == 0 && val == OPT_KEY1) {
            s->optkey_state = 1;
        } else if (s->optkey_state == 1 && val == OPT_KEY2) {
            FL(s, FL_CR) &= ~CR_OPTLOCK;
            s->optkey_state = 0;
        } else {
            s->optkey_state = 0;
        }
        return;
    case FL_SR:
        FL(s, FL_SR) &= ~(val & SR_W1C);
        fl_update_irq(s);
        return;
    case FL_CR: {
        uint32_t old = FL(s, FL_CR);

        if (val & CR_LOCK) {
            FL(s, FL_CR) = old | CR_LOCK;
            if (!(old & CR_LOCK)) {
                s->pg_pending = false;
                s->pg_have = 0;
            }
            return;
        }
        if (old & CR_LOCK) {
            qemu_log_mask(LOG_GUEST_ERROR, "stm32g4 flash: CR write while locked\n");
            return;
        }
        if ((val & CR_STRT) && (val & (CR_PER | CR_MER1 | CR_MER2))) {
            flash_erase(s, val);
        }
        if ((val & CR_OPTSTRT) && !(old & CR_OPTLOCK)) {
            qemu_log_mask(LOG_UNIMP, "stm32g4 flash: option byte programming ignored\n");
            FL(s, FL_SR) |= SR_EOP;
        }
        if (val & CR_OBL_LAUNCH) {
            qemu_log_mask(LOG_GUEST_ERROR, "stm32g4 flash: option byte launch reset\n");
            RCC(s, RCC_CSR) |= CSR_OBLRSTF;
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        val &= ~(CR_STRT | CR_OPTSTRT | CR_OBL_LAUNCH);
        if (!(val & CR_PG) && s->pg_pending) {
            s->pg_pending = false;
            s->pg_have = 0;
        }
        FL(s, FL_CR) = val | (old & CR_OPTLOCK);
        fl_update_irq(s);
        return;
    }
    case FL_OPTR:
        return;
    }
    if (off < sizeof(s->fl)) {
        s->fl[off / 4] = val;
    }
}

#define OPS(name)                                                       \
    static const MemoryRegionOps name##_ops = {                         \
        .read = name##_read,                                            \
        .write = name##_write,                                          \
        .endianness = DEVICE_LITTLE_ENDIAN,                             \
        .impl = { .min_access_size = 4, .max_access_size = 4 },         \
        .valid = { .min_access_size = 1, .max_access_size = 4 },        \
    };
OPS(rcc)
OPS(pwr)
OPS(flash_if)

static bool powered_on;

static void stm32g4_sysctrl_reset(DeviceState *dev)
{
    Stm32g4SysctrlState *s = STM32G4_SYSCTRL(dev);
    uint32_t csr = powered_on ? RCC(s, RCC_CSR) & CSR_FLAGS : 0;
    uint32_t h7cause = stm32h7_sysctrl_get_reset_cause();
    uint32_t cause;

    if (!powered_on) {
        cause = CSR_PINRSTF | CSR_BORRSTF;
    } else if (h7cause == STM32H7_RSR_IWDG1RSTF) {
        cause = CSR_PINRSTF | CSR_IWDGRSTF;
    } else {
        cause = CSR_PINRSTF | CSR_SFTRSTF;
    }
    if (powered_on) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32g4: system reset (%s)\n",
                      (cause & CSR_IWDGRSTF) ? "IWDG watchdog" : "software");
    }
    stm32h7_sysctrl_set_reset_cause(0);
    powered_on = true;

    memset(s->rcc, 0, sizeof(s->rcc));
    memset(s->pwr, 0, sizeof(s->pwr));
    memset(s->fl, 0, sizeof(s->fl));
    s->sws = 1;
    RCC(s, RCC_CR) = CR_HSION | BIT(9);
    RCC(s, RCC_ICSCR) = 0x40000000;
    RCC(s, RCC_PLLCFGR) = 0x00001000;
    RCC(s, RCC_CRRCR) = 0;
    RCC(s, RCC_CSR) = csr | cause;
    s->pwr[PWR_CR1 / 4] = 0x00000200;
    s->pwr[0x08 / 4] = 0x00008000;
    s->pwr[PWR_CR5 / 4] = 0x00000100;
    FL(s, FL_ACR) = 0x00000601;
    FL(s, FL_CR) = CR_LOCK | CR_OPTLOCK;
    s->key_state = s->optkey_state = 0;
    s->pg_pending = false;
    s->pg_have = 0;
    update_clocks(s);
    qemu_set_irq(s->flash_irq, 0);
}

static void stm32g4_sysctrl_realize(DeviceState *dev, Error **errp)
{
    Stm32g4SysctrlState *s = STM32G4_SYSCTRL(dev);

    if (!memory_region_init_rom_device(&s->flash_mem, OBJECT(dev), &flash_array_ops, s,
                                       "stm32g4.flash", s->flash_size, errp)) {
        return;
    }
    memset(memory_region_get_ram_ptr(&s->flash_mem), 0xFF, s->flash_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->flash_mem);   /* mmio 3 */
}

uint8_t *stm32g4_sysctrl_flash_ptr(Stm32g4SysctrlState *s)
{
    return memory_region_get_ram_ptr(&s->flash_mem);
}

static void stm32g4_sysctrl_init(Object *obj)
{
    Stm32g4SysctrlState *s = STM32G4_SYSCTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->rcc_mmio, obj, &rcc_ops, s, "stm32g4-rcc", 0x400);
    memory_region_init_io(&s->pwr_mmio, obj, &pwr_ops, s, "stm32g4-pwr", 0x400);
    memory_region_init_io(&s->flash_if_mmio, obj, &flash_if_ops, s, "stm32g4-flash-if", 0x400);
    sysbus_init_mmio(sbd, &s->rcc_mmio);
    sysbus_init_mmio(sbd, &s->pwr_mmio);
    sysbus_init_mmio(sbd, &s->flash_if_mmio);
    sysbus_init_irq(sbd, &s->flash_irq);
    s->hclk = qdev_init_clock_out(DEVICE(obj), "hclk");
    clock_set_hz(s->hclk, HSI16_HZ);
}

static const Property stm32g4_sysctrl_props[] = {
    DEFINE_PROP_UINT32("flash-size", Stm32g4SysctrlState, flash_size, 512 * 1024),
    /* HSE crystal/oscillator frequency (a board property) */
    DEFINE_PROP_UINT32("hse-hz", Stm32g4SysctrlState, hse_hz, 8000000),
    /* FLASH_OPTR: factory default, dual bank (DBANK = 1), RDP level 0 */
    DEFINE_PROP_UINT32("optr", Stm32g4SysctrlState, optr, 0xFFEFF8AA),
};

static const VMStateDescription vmstate_stm32g4_sysctrl = {
    .name = TYPE_STM32G4_SYSCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(rcc, Stm32g4SysctrlState, 0x100 / 4),
        VMSTATE_UINT32_ARRAY(pwr, Stm32g4SysctrlState, 0x100 / 4),
        VMSTATE_UINT32_ARRAY(fl, Stm32g4SysctrlState, 0x80 / 4),
        VMSTATE_UINT32(sws, Stm32g4SysctrlState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32g4_sysctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32g4_sysctrl_realize;
    device_class_set_legacy_reset(dc, stm32g4_sysctrl_reset);
    device_class_set_props(dc, stm32g4_sysctrl_props);
    dc->vmsd = &vmstate_stm32g4_sysctrl;
}

static const TypeInfo stm32g4_sysctrl_info = {
    .name = TYPE_STM32G4_SYSCTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32g4SysctrlState),
    .instance_init = stm32g4_sysctrl_init,
    .class_init = stm32g4_sysctrl_class_init,
};

static void stm32g4_sysctrl_register(void)
{
    type_register_static(&stm32g4_sysctrl_info);
}
type_init(stm32g4_sysctrl_register)
