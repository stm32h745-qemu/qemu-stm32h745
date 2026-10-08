/*
 * STM32H7 system control: RCC, PWR, flash interface, HSEM.
 * See include/hw/misc/stm32h7_sysctrl.h. Register offsets from RM0399
 * (STM32H745/755 and H747/757).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/stm32h7_sysctrl.h"
#include "migration/vmstate.h"
#include "exec/memory.h"

/* ---- RCC ---- */
#define RCC_CR       0x00
#define RCC_CFGR     0x10
#define RCC_BDCR     0x70
#define RCC_CSR      0x74
#define RCC_GCR      0xA0
#define RCC_RSR      0xD0
#define RCC_C1_RSR   0x130
#define RCC_C2_RSR   0x190

#define CR_HSION     BIT(0)
#define CR_HSIRDY    BIT(2)
#define CR_CSION     BIT(7)
#define CR_CSIRDY    BIT(8)
#define CR_HSI48ON   BIT(12)
#define CR_HSI48RDY  BIT(13)
#define CR_D1CKRDY   BIT(14)
#define CR_D2CKRDY   BIT(15)
#define CR_HSEON     BIT(16)
#define CR_HSERDY    BIT(17)
#define CR_PLL1ON    BIT(24)
#define CR_PLL1RDY   BIT(25)
#define CR_PLL2ON    BIT(26)
#define CR_PLL2RDY   BIT(27)
#define CR_PLL3ON    BIT(28)
#define CR_PLL3RDY   BIT(29)
#define GCR_BOOT_C2  BIT(3)
#define RSR_RMVF     BIT(16)

/* ---- PWR ---- */
#define PWR_CSR1     0x04
#define PWR_CR3      0x0C
#define PWR_D3CR     0x18
#define CSR1_ACTVOSRDY BIT(13)
#define CR3_USB33DEN BIT(24)
#define CR3_USB33RDY BIT(26)
#define D3CR_VOSRDY  BIT(13)
#define D3CR_VOS_MASK (3u << 14)

/* ---- Flash interface (per bank: bank 2 registers at +0x100) ---- */
#define FL_ACR       0x00
#define FL_KEYR      0x04
#define FL_OPTKEYR   0x08
#define FL_CR        0x0C
#define FL_SR        0x10
#define FL_CCR       0x14
#define FL_OPTCR     0x18
#define FL_OPTSR_CUR 0x1C
#define FL_OPTSR_PRG 0x20
#define FL_BOOT4_CUR 0x4C
#define FL_BANK2     0x100
#define FLCR_LOCK    BIT(0)
#define FLCR_SER     BIT(2)
#define FLCR_BER     BIT(3)
#define FLCR_START   BIT(7)
#define FLCR_SNB_SHIFT 8
#define FLSR_EOP     BIT(16)
#define OPTCR_OPTLOCK BIT(0)
#define OPTCR_OPTSTART BIT(1)
#define FLASH_KEY1   0x45670123
#define FLASH_KEY2   0xCDEF89AB
#define OPT_KEY1     0x08192A3B
#define OPT_KEY2     0x4C5D6E7F
#define FLASH_SECTOR (128 * 1024)

/* ---- HSEM ---- */
#define HSEM_R       0x000
#define HSEM_RLR     0x080
#define HSEM_C1IER   0x100
#define HSEM_C1ICR   0x104
#define HSEM_C1ISR   0x108
#define HSEM_C1MISR  0x10C
#define HSEM_C2IER   0x110
#define HSEM_C2ICR   0x114
#define HSEM_C2ISR   0x118
#define HSEM_C2MISR  0x11C
#define HSEM_CR      0x140
#define HSEM_KEYR    0x144
#define HSEM_LOCK    BIT(31)
#define HSEM_COREID_SHIFT 8
#define COREID_CM7   3
#define COREID_CM4   1

static uint64_t rcc_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    uint32_t v = s->rcc[off / 4];

    switch (off) {
    case RCC_CR:
        v &= ~(CR_HSIRDY | CR_CSIRDY | CR_HSI48RDY | CR_HSERDY | CR_PLL1RDY |
               CR_PLL2RDY | CR_PLL3RDY);
        v |= CR_D1CKRDY | CR_D2CKRDY;
        v |= (v & CR_HSION) ? CR_HSIRDY : 0;
        v |= (v & CR_CSION) ? CR_CSIRDY : 0;
        v |= (v & CR_HSI48ON) ? CR_HSI48RDY : 0;
        v |= (v & CR_HSEON) ? CR_HSERDY : 0;
        v |= (v & CR_PLL1ON) ? CR_PLL1RDY : 0;
        v |= (v & CR_PLL2ON) ? CR_PLL2RDY : 0;
        v |= (v & CR_PLL3ON) ? CR_PLL3RDY : 0;
        break;
    case RCC_CFGR:          /* SWS follows SW */
        v = (v & ~(7u << 3)) | ((v & 7u) << 3);
        break;
    case RCC_BDCR:          /* LSERDY follows LSEON */
        v = (v & ~BIT(1)) | ((v & BIT(0)) << 1);
        break;
    case RCC_CSR:           /* LSIRDY follows LSION */
        v = (v & ~BIT(1)) | ((v & BIT(0)) << 1);
        break;
    case RCC_C1_RSR:
    case RCC_C2_RSR:
        v = s->rcc[RCC_RSR / 4];
        break;
    }
    return v;
}

static void rcc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;

    switch (off) {
    case RCC_RSR:
    case RCC_C1_RSR:
    case RCC_C2_RSR:
        if (val & RSR_RMVF) {
            s->rcc[RCC_RSR / 4] = 0;
        }
        return;
    case RCC_GCR:
        if ((val & GCR_BOOT_C2) && !(s->rcc[off / 4] & GCR_BOOT_C2)) {
            qemu_set_irq(s->cm4_boot, 1);
        }
        break;
    }
    s->rcc[off / 4] = val;
}

static uint64_t pwr_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    uint32_t v = s->pwr[off / 4];

    switch (off) {
    case PWR_CSR1:          /* voltage scaling done, reports the requested VOS */
        v = (v & ~(CSR1_ACTVOSRDY | (3u << 14))) | CSR1_ACTVOSRDY |
            (s->pwr[PWR_D3CR / 4] & D3CR_VOS_MASK);
        break;
    case PWR_CR3:
        v = (v & ~CR3_USB33RDY) | ((v & CR3_USB33DEN) ? CR3_USB33RDY : 0);
        break;
    case PWR_D3CR:
        v |= D3CR_VOSRDY;
        break;
    }
    return v;
}

static void pwr_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    s->pwr[off / 4] = val;
}

static void flash_erase(Stm32h7SysctrlState *s, int bank, uint32_t cr)
{
    uint8_t *mem;
    uint32_t bank_size = s->flash_size / 2;
    uint32_t start, len;

    if (!s->flash_mem) {
        return;
    }
    mem = memory_region_get_ram_ptr(s->flash_mem);
    if (cr & FLCR_BER) {
        start = bank * bank_size;
        len = bank_size;
    } else {
        start = bank * bank_size + ((cr >> FLCR_SNB_SHIFT) & 7) * FLASH_SECTOR;
        len = FLASH_SECTOR;
    }
    if (start + len <= s->flash_size) {
        memset(mem + start, 0xFF, len);
        memory_region_set_dirty(s->flash_mem, start, len);
    }
}

static uint64_t flash_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    hwaddr reg = off & 0xFF;

    switch (reg) {
    case FL_SR:             /* never busy; EOP latched until cleared */
        return s->flash[off / 4] & FLSR_EOP;
    case FL_OPTSR_CUR:
    case FL_OPTSR_PRG:
        return s->optsr;
    case FL_BOOT4_CUR:
        return s->boot_cm4_add;
    }
    return s->flash[off / 4];
}

static void flash_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    int bank = off >= FL_BANK2 ? 1 : 0;
    hwaddr reg = off & 0xFF;
    hwaddr bank_base = bank ? FL_BANK2 : 0;

    switch (reg) {
    case FL_KEYR:
        if (s->flash_key_state[bank] == 0 && val == FLASH_KEY1) {
            s->flash_key_state[bank] = 1;
        } else if (s->flash_key_state[bank] == 1 && val == FLASH_KEY2) {
            s->flash[(bank_base + FL_CR) / 4] &= ~FLCR_LOCK;
            s->flash_key_state[bank] = 0;
        } else {
            s->flash_key_state[bank] = 0;
        }
        return;
    case FL_OPTKEYR:
        if (s->opt_key_state == 0 && val == OPT_KEY1) {
            s->opt_key_state = 1;
        } else if (s->opt_key_state == 1 && val == OPT_KEY2) {
            s->flash[FL_OPTCR / 4] &= ~OPTCR_OPTLOCK;
            s->opt_key_state = 0;
        } else {
            s->opt_key_state = 0;
        }
        return;
    case FL_CR:
        if (s->flash[off / 4] & FLCR_LOCK) {
            qemu_log_mask(LOG_GUEST_ERROR, "stm32h7 flash: CR%d write while locked\n",
                          bank + 1);
            return;
        }
        if (val & FLCR_START) {
            flash_erase(s, bank, val);
            s->flash[(bank_base + FL_SR) / 4] |= FLSR_EOP;
            val &= ~FLCR_START;
        }
        s->flash[off / 4] = val;
        return;
    case FL_CCR:            /* write 1 to clear status */
        s->flash[(bank_base + FL_SR) / 4] &= ~val;
        return;
    case FL_OPTCR:
        if (val & OPTCR_OPTSTART) {
            qemu_log_mask(LOG_UNIMP, "stm32h7 flash: option byte programming ignored\n");
            val &= ~OPTCR_OPTSTART;
        }
        break;
    }
    s->flash[off / 4] = val;
}

static int hsem_core(void)
{
    /* Bus master id of the access: the M4 runs as CPU index 1 */
    return (current_cpu && current_cpu->cpu_index == 1) ? COREID_CM4 : COREID_CM7;
}

static void hsem_update_irq(Stm32h7SysctrlState *s)
{
    qemu_set_irq(s->hsem_irq[0], !!(s->hsem_isr[0] & s->hsem_ier[0]));
    qemu_set_irq(s->hsem_irq[1], !!(s->hsem_isr[1] & s->hsem_ier[1]));
}

static void hsem_freed(Stm32h7SysctrlState *s, int i)
{
    s->hsem_r[i] = 0;
    s->hsem_isr[0] |= BIT(i);
    s->hsem_isr[1] |= BIT(i);
    hsem_update_irq(s);
}

static uint64_t hsem_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    int i;

    if (off < HSEM_RLR) {
        return s->hsem_r[off / 4];
    }
    if (off < HSEM_C1IER) {             /* 1-step lock: read locks if free */
        i = (off - HSEM_RLR) / 4;
        if (!(s->hsem_r[i] & HSEM_LOCK)) {
            s->hsem_r[i] = HSEM_LOCK | (hsem_core() << HSEM_COREID_SHIFT);
        }
        return s->hsem_r[i];
    }
    switch (off) {
    case HSEM_C1IER:  return s->hsem_ier[0];
    case HSEM_C1ISR:  return s->hsem_isr[0];
    case HSEM_C1MISR: return s->hsem_isr[0] & s->hsem_ier[0];
    case HSEM_C2IER:  return s->hsem_ier[1];
    case HSEM_C2ISR:  return s->hsem_isr[1];
    case HSEM_C2MISR: return s->hsem_isr[1] & s->hsem_ier[1];
    case HSEM_KEYR:   return s->hsem_keyr;
    }
    return 0;
}

static void hsem_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Stm32h7SysctrlState *s = opaque;
    int i;

    if (off < HSEM_RLR) {               /* 2-step lock / unlock */
        i = off / 4;
        if (val & HSEM_LOCK) {
            if (!(s->hsem_r[i] & HSEM_LOCK)) {
                s->hsem_r[i] = HSEM_LOCK | (hsem_core() << HSEM_COREID_SHIFT) | (val & 0xFF);
            }
        } else if ((s->hsem_r[i] & HSEM_LOCK) &&
                   ((s->hsem_r[i] >> HSEM_COREID_SHIFT) & 0xF) == hsem_core() &&
                   (s->hsem_r[i] & 0xFF) == (val & 0xFF)) {
            hsem_freed(s, i);
        }
        return;
    }
    switch (off) {
    case HSEM_C1IER: s->hsem_ier[0] = val; break;
    case HSEM_C1ICR: s->hsem_isr[0] &= ~val; break;
    case HSEM_C2IER: s->hsem_ier[1] = val; break;
    case HSEM_C2ICR: s->hsem_isr[1] &= ~val; break;
    case HSEM_CR:                       /* clear all of a core's semaphores */
        if ((val >> 16) == (s->hsem_keyr >> 16)) {
            for (i = 0; i < STM32H7_HSEM_COUNT; i++) {
                if ((s->hsem_r[i] & HSEM_LOCK) &&
                    ((s->hsem_r[i] >> HSEM_COREID_SHIFT) & 0xF) == ((val >> 8) & 0xF)) {
                    hsem_freed(s, i);
                }
            }
        }
        break;
    case HSEM_KEYR: s->hsem_keyr = val; break;
    }
    hsem_update_irq(s);
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
OPS(flash)
OPS(hsem)

/* Cause of the next system reset, for RCC_RSR (set by the IWDG model);
 * any other reset after power-on is a software reset (AIRCR SYSRESETREQ) */
static uint32_t pending_reset_cause;
static bool powered_on;

void stm32h7_sysctrl_set_reset_cause(uint32_t rsr_flag)
{
    pending_reset_cause = rsr_flag;
}

static void stm32h7_sysctrl_reset(DeviceState *dev)
{
    Stm32h7SysctrlState *s = STM32H7_SYSCTRL(dev);
    /* RSR keeps its flags until firmware writes RMVF; every reset adds
     * PINRSTF (NRST is driven) and its cause */
    uint32_t rsr = powered_on ? s->rcc[RCC_RSR / 4] : 0;
    uint32_t cause = powered_on ? (pending_reset_cause ? pending_reset_cause
                                                       : STM32H7_RSR_SFTRSTF)
                                : STM32H7_RSR_PORRSTF | STM32H7_RSR_BORRSTF;

    memset(s->rcc, 0, sizeof(s->rcc));
    memset(s->pwr, 0, sizeof(s->pwr));
    memset(s->flash, 0, sizeof(s->flash));
    s->rcc[RCC_CR / 4] = CR_HSION | BIT(5);         /* HSI on, HSIDIVF */
    s->rcc[0x28 / 4] = 0x02020200;                  /* PLLCKSELR */
    s->rcc[0x2C / 4] = 0x01FF0000;                  /* PLLCFGR */
    s->rcc[0x30 / 4] = 0x01010280;                  /* PLL1DIVR */
    s->rcc[RCC_RSR / 4] = rsr | cause | STM32H7_RSR_PINRSTF;
    if (powered_on) {
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7: system reset (%s)\n",
                      cause == STM32H7_RSR_IWDG1RSTF ? "IWDG1 watchdog" : "software");
    }
    powered_on = true;
    pending_reset_cause = 0;
    s->pwr[0x00 / 4] = 0xF000C000;                  /* CR1 */
    s->pwr[PWR_CR3 / 4] = 0x00000006;               /* SDEN | LDOEN */
    s->pwr[PWR_D3CR / 4] = 1u << 14;                /* VOS3 */
    s->flash[FL_ACR / 4] = 0x37;
    s->flash[FL_CR / 4] = FLCR_LOCK | 0x30;
    s->flash[(FL_BANK2 + FL_CR) / 4] = FLCR_LOCK | 0x30;
    s->flash[FL_OPTCR / 4] = OPTCR_OPTLOCK;
    s->flash_key_state[0] = s->flash_key_state[1] = s->opt_key_state = 0;
    memset(s->hsem_r, 0, sizeof(s->hsem_r));
    s->hsem_ier[0] = s->hsem_ier[1] = 0;
    s->hsem_isr[0] = s->hsem_isr[1] = 0;
    s->hsem_keyr = 0;
}

static void stm32h7_sysctrl_init(Object *obj)
{
    Stm32h7SysctrlState *s = STM32H7_SYSCTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->rcc_mmio, obj, &rcc_ops, s, "stm32h7-rcc", 0x400);
    memory_region_init_io(&s->pwr_mmio, obj, &pwr_ops, s, "stm32h7-pwr", 0x400);
    memory_region_init_io(&s->flash_mmio, obj, &flash_ops, s, "stm32h7-flash-if", 0x1000);
    memory_region_init_io(&s->hsem_mmio, obj, &hsem_ops, s, "stm32h7-hsem", 0x400);
    sysbus_init_mmio(sbd, &s->rcc_mmio);
    sysbus_init_mmio(sbd, &s->pwr_mmio);
    sysbus_init_mmio(sbd, &s->flash_mmio);
    sysbus_init_mmio(sbd, &s->hsem_mmio);
    sysbus_init_irq(sbd, &s->hsem_irq[0]);
    sysbus_init_irq(sbd, &s->hsem_irq[1]);
    qdev_init_gpio_out_named(DEVICE(obj), &s->cm4_boot, "cm4-boot", 1);
}

static const Property stm32h7_sysctrl_props[] = {
    DEFINE_PROP_LINK("flash", Stm32h7SysctrlState, flash_mem, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("flash-size", Stm32h7SysctrlState, flash_size, 2 * 1024 * 1024),
    /* Factory defaults: BCM7 = 1, BCM4 = 1, RDP level 0 (0xAA). Override with
     * -global stm32h7-sysctrl.optsr=... to match programmed option bytes. */
    DEFINE_PROP_UINT32("optsr", Stm32h7SysctrlState, optsr, BIT(22) | BIT(21) | (0xAA << 8)),
    /* FLASH_BOOT4_CUR: BOOT_CM4_ADD1 (31:16) = 0x1000, BOOT_CM4_ADD0 (15:0) =
     * 0x0810, i.e. the M4 boots from 0x08100000 (factory default) */
    DEFINE_PROP_UINT32("boot-cm4-add", Stm32h7SysctrlState, boot_cm4_add, 0x10000810),
};

static const VMStateDescription vmstate_stm32h7_sysctrl = {
    .name = TYPE_STM32H7_SYSCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(rcc, Stm32h7SysctrlState, 0x400 / 4),
        VMSTATE_UINT32_ARRAY(pwr, Stm32h7SysctrlState, 0x400 / 4),
        VMSTATE_UINT32_ARRAY(flash, Stm32h7SysctrlState, 0x1000 / 4),
        VMSTATE_UINT32_ARRAY(hsem_r, Stm32h7SysctrlState, STM32H7_HSEM_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32h7_sysctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, stm32h7_sysctrl_reset);
    device_class_set_props(dc, stm32h7_sysctrl_props);
    dc->vmsd = &vmstate_stm32h7_sysctrl;
}

static const TypeInfo stm32h7_sysctrl_info = {
    .name = TYPE_STM32H7_SYSCTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7SysctrlState),
    .instance_init = stm32h7_sysctrl_init,
    .class_init = stm32h7_sysctrl_class_init,
};

static void stm32h7_sysctrl_register(void)
{
    type_register_static(&stm32h7_sysctrl_info);
}
type_init(stm32h7_sysctrl_register)

uint64_t stm32h7_sysctrl_apb_hz(Stm32h7SysctrlState *s, int apb, uint64_t core_hz)
{
    static const unsigned hdiv[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
    uint32_t d1 = s->rcc[0x18 / 4], d2 = s->rcc[0x1C / 4], d3 = s->rcc[0x20 / 4];
    uint32_t hpre = d1 & 0xF, ppre;
    uint64_t hclk = (hpre & 8) ? core_hz / hdiv[hpre & 7] : core_hz;

    switch (apb) {
    case 1: ppre = (d2 >> 4) & 7; break;
    case 2: ppre = (d2 >> 8) & 7; break;
    case 3: ppre = (d1 >> 4) & 7; break;
    default: ppre = (d3 >> 4) & 7; break;
    }
    return (ppre & 4) ? hclk >> ((ppre & 3) + 1) : hclk;
}
