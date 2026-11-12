/*
 * STM32H7 FDCAN (Bosch M_CAN) with an SLCAN chardev as the bus.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_NET_STM32H7_FDCAN_H
#define HW_NET_STM32H7_FDCAN_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_STM32H7_FDCAN "stm32h7-fdcan"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7FdcanState, STM32H7_FDCAN)

struct Stm32h7FdcanState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion *msg_ram;      /* shared message RAM (10 KB on H7) */
    CharBackend chr;            /* SLCAN text frames */
    qemu_irq irq[2];            /* interrupt line 0 / line 1 */

    bool g4;                    /* STM32G4 register map and fixed message RAM layout */
    uint32_t ram_offset;        /* G4: this instance's part of the shared SRAMCAN */
    uint32_t ckdiv;             /* G4: clock calibration unit divider (FDCAN_CKDIV) */
    uint32_t regs[0x100 / 4];
    char line[40];
    uint32_t line_len;
    uint64_t tx_frames, rx_frames, rx_dropped;
    /* RX pacing ("rx-frame-ns" > 0): frames from the chardev wait here and
     * enter the controller at most one per frame time on the virtual clock,
     * as frames on a real bus do; 0: at once (the default) */
    uint32_t rx_frame_ns;
    QEMUTimer *rx_timer;
    char rxq[64][40];
    uint8_t rxq_len[64];
    uint32_t rxq_head, rxq_count;
    int64_t rx_next_ns;
};

#endif
