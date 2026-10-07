/*
 * STM32H7 USART/UART with DMA requests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_CHAR_STM32H7_USART_H
#define HW_CHAR_STM32H7_USART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qom/object.h"

#define TYPE_STM32H7_USART "stm32h7-usart"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7UsartState, STM32H7_USART)

struct Stm32h7UsartState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dma_rx, dma_tx;
    Clock *clk;
    CharBackend chr;
    guint watch_tag;
    QEMUTimer *rx_timer;
    Fifo8 rxq;
    bool idle_armed;

    uint32_t cr1, cr2, cr3, brr, gtpr, rtor, isr, rdr, tdr, presc;

    /* kernel clock in Hz, if set by the SoC (else the "clk" input) */
    uint64_t (*kernel_hz)(void *opaque);
    void *kernel_hz_opaque;
};

#endif
