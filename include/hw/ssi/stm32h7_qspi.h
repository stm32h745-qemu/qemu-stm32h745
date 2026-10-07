/*
 * STM32H7 QUADSPI controller with an attached serial NOR flash.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SSI_STM32H7_QSPI_H
#define HW_SSI_STM32H7_QSPI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32H7_QSPI "stm32h7-qspi"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7QspiState, STM32H7_QSPI)

#define STM32H7_QSPI_FIFO 32

struct Stm32h7QspiState {
    SysBusDevice parent_obj;

    MemoryRegion regs;
    MemoryRegion mapped;            /* memory-mapped read window */
    qemu_irq irq;
    BlockBackend *blk;              /* optional backing image (raw) */

    uint32_t flash_size;
    uint8_t *storage;

    /* registers */
    uint32_t cr, dcr, sr, dlr, ccr, ar, abr, psmkr, psmar, pir, lptr;

    /* transfer in progress */
    bool active;
    uint32_t cmd;                   /* instruction */
    uint32_t addr;
    uint32_t len;                   /* data bytes in the transfer */
    uint32_t done;                  /* data bytes moved so far */
    uint8_t buf[512];               /* indirect write buffer / read source */
    uint8_t fifo[STM32H7_QSPI_FIFO];
    uint32_t fifo_level;

    /* flash state */
    uint8_t sr1, sr2, sr3;
    bool wel, addr4;
};

#endif
