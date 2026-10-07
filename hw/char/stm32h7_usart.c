/*
 * STM32H7 USART/UART (RM0399 chapter 53), asynchronous mode.
 *
 * Bytes from the chardev are queued and delivered one per character time at
 * the programmed baud rate (BRR, PRESC, OVER8, 10 bits per character), so
 * firmware sees a real line rate. A byte is held while the previous one is
 * unread, so emulation latency never causes an overrun (ORE is not raised). IDLE is flagged one character time after
 * the line goes quiet (once per idle period, after at least one byte), and
 * the receiver timeout (RTOEN/RTOR) the same way. Transmission is
 * immediate (TXE/TC are set once the chardev has taken the byte).
 *
 * DMA: GPIO output 0 is the RX request (RXNE while CR3.DMAR), output 1 the TX
 * request (TXE while CR3.DMAT); wire them to a DMAMUX request input. Reading
 * RDR / writing TDR from the DMA clears them as on hardware.
 *
 * Modeled: UE/RE/TE, TXE/TC/RXNE/ORE/IDLE/RTOF and their interrupts, ICR,
 * RQR (RXFRQ/TXFRQ), DMA requests, CTS treated as always asserted.
 * Not modeled: the FIFOs (FIFOEN is ignored), 7/9-bit data, parity and
 * framing errors, LIN, smartcard, IrDA, synchronous mode, auto baud rate.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/fifo8.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/char/stm32h7_usart.h"

#define R_CR1   0x00
#define R_CR2   0x04
#define R_CR3   0x08
#define R_BRR   0x0C
#define R_GTPR  0x10
#define R_RTOR  0x14
#define R_RQR   0x18
#define R_ISR   0x1C
#define R_ICR   0x20
#define R_RDR   0x24
#define R_TDR   0x28
#define R_PRESC 0x2C

#define CR1_UE     BIT(0)
#define CR1_RE     BIT(2)
#define CR1_TE     BIT(3)
#define CR1_IDLEIE BIT(4)
#define CR1_RXNEIE BIT(5)
#define CR1_TCIE   BIT(6)
#define CR1_TXEIE  BIT(7)
#define CR1_PEIE   BIT(8)
#define CR1_OVER8  BIT(15)
#define CR1_RTOIE  BIT(26)

#define CR2_RTOEN  BIT(23)

#define CR3_EIE    BIT(0)
#define CR3_DMAR   BIT(6)
#define CR3_DMAT   BIT(7)
#define CR3_CTSIE  BIT(10)

#define ISR_PE    BIT(0)
#define ISR_FE    BIT(1)
#define ISR_NE    BIT(2)
#define ISR_ORE   BIT(3)
#define ISR_IDLE  BIT(4)
#define ISR_RXNE  BIT(5)
#define ISR_TC    BIT(6)
#define ISR_TXE   BIT(7)
#define ISR_CTSIF BIT(9)
#define ISR_CTS   BIT(10)
#define ISR_RTOF  BIT(11)
#define ISR_BUSY  BIT(16)
#define ISR_TEACK BIT(21)
#define ISR_REACK BIT(22)

#define RQR_RXFRQ BIT(3)
#define RQR_TXFRQ BIT(4)

#define RXQ_SIZE 4096

static const unsigned presc_div[12] = { 1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256 };

static int64_t char_ns(Stm32h7UsartState *s)
{
    uint64_t hz = s->kernel_hz ? s->kernel_hz(s->kernel_hz_opaque) : clock_get_hz(s->clk);
    uint32_t brr = s->brr & 0xFFFF;
    uint64_t baud;

    if (!hz || brr < 16) {
        return 10 * NANOSECONDS_PER_SECOND / 115200;
    }
    hz /= presc_div[MIN(s->presc & 0xF, 11)];
    if (s->cr1 & CR1_OVER8) {
        brr = (brr & ~0xF) | ((brr & 7) << 1);
        baud = 2 * hz / brr;
    } else {
        baud = hz / brr;
    }
    return baud ? 10 * NANOSECONDS_PER_SECOND / baud : 1;
}

static void update(Stm32h7UsartState *s)
{
    uint32_t i = s->isr, c1 = s->cr1, c3 = s->cr3;
    bool lvl = ((i & ISR_TXE) && (c1 & CR1_TXEIE)) ||
               ((i & ISR_TC) && (c1 & CR1_TCIE)) ||
               ((i & ISR_RXNE) && (c1 & CR1_RXNEIE)) ||
               ((i & ISR_ORE) && ((c1 & CR1_RXNEIE) || (c3 & CR3_EIE))) ||
               ((i & ISR_IDLE) && (c1 & CR1_IDLEIE)) ||
               ((i & ISR_RTOF) && (c1 & CR1_RTOIE)) ||
               ((i & ISR_CTSIF) && (c3 & CR3_CTSIE));

    qemu_set_irq(s->irq, lvl);
    qemu_set_irq(s->dma_rx, (i & ISR_RXNE) && (c3 & CR3_DMAR) && (c1 & CR1_UE));
    qemu_set_irq(s->dma_tx, (i & ISR_TXE) && (c3 & CR3_DMAT) && (c1 & CR1_UE) &&
                            (c1 & CR1_TE));
}

static bool rx_enabled(Stm32h7UsartState *s)
{
    return (s->cr1 & (CR1_UE | CR1_RE)) == (CR1_UE | CR1_RE);
}

/* Deliver the next queued byte, then come back one character time later */
static void rx_tick(void *opaque)
{
    Stm32h7UsartState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!rx_enabled(s)) {
        return;
    }
    if (!fifo8_is_empty(&s->rxq)) {
        if (s->isr & ISR_RXNE) {
            /* The previous byte is still unread: hold this one (emulation
             * latency is not line timing; hardware has a 16-byte FIFO) */
            timer_mod(s->rx_timer, now + char_ns(s));
            return;
        }
        s->rdr = fifo8_pop(&s->rxq);
        s->isr |= ISR_RXNE;
        s->idle_armed = true;
        timer_mod(s->rx_timer, now + char_ns(s));
        qemu_chr_fe_accept_input(&s->chr);
    } else if (s->idle_armed) {
        s->idle_armed = false;
        s->isr |= ISR_IDLE;
        if (s->cr2 & CR2_RTOEN) {
            s->isr |= ISR_RTOF;
        }
    }
    update(s);
}

static int can_receive(void *opaque)
{
    Stm32h7UsartState *s = opaque;

    return rx_enabled(s) ? fifo8_num_free(&s->rxq) : 0;
}

static void receive(void *opaque, const uint8_t *buf, int size)
{
    Stm32h7UsartState *s = opaque;
    bool idle = fifo8_is_empty(&s->rxq) && !timer_pending(s->rx_timer);

    fifo8_push_all(&s->rxq, buf, MIN(size, fifo8_num_free(&s->rxq)));
    if (idle) {
        rx_tick(s);
    }
}

static gboolean tx_ready(void *do_not_use, GIOCondition cond, void *opaque);

static void tx_byte(Stm32h7UsartState *s)
{
    uint8_t ch = s->tdr;

    if (qemu_chr_fe_backend_connected(&s->chr) &&
        qemu_chr_fe_write(&s->chr, &ch, 1) <= 0) {
        s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                             tx_ready, s);
        if (s->watch_tag) {
            return;                     /* retry when the backend drains */
        }
    }
    s->isr |= ISR_TXE | ISR_TC;
    update(s);
}

static gboolean tx_ready(void *do_not_use, GIOCondition cond, void *opaque)
{
    Stm32h7UsartState *s = opaque;

    s->watch_tag = 0;
    tx_byte(s);
    return G_SOURCE_REMOVE;
}

static uint64_t usart_read(void *opaque, hwaddr off, unsigned size)
{
    Stm32h7UsartState *s = opaque;
    uint64_t v = 0;

    switch (off) {
    case R_CR1:   v = s->cr1; break;
    case R_CR2:   v = s->cr2; break;
    case R_CR3:   v = s->cr3; break;
    case R_BRR:   v = s->brr; break;
    case R_GTPR:  v = s->gtpr; break;
    case R_RTOR:  v = s->rtor; break;
    case R_ISR:   v = s->isr | ISR_CTS; break;
    case R_RDR:
        v = s->rdr;
        s->isr &= ~ISR_RXNE;
        update(s);
        break;
    case R_TDR:   v = s->tdr; break;
    case R_PRESC: v = s->presc; break;
    }
    return v;
}

static void usart_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Stm32h7UsartState *s = opaque;

    switch (off) {
    case R_CR1:
        s->cr1 = v;
        s->isr = (s->isr & ~(ISR_TEACK | ISR_REACK)) |
                 ((v & CR1_TE) ? ISR_TEACK : 0) | ((v & CR1_RE) ? ISR_REACK : 0);
        if (rx_enabled(s)) {
            qemu_chr_fe_accept_input(&s->chr);
            if (!fifo8_is_empty(&s->rxq) && !timer_pending(s->rx_timer)) {
                rx_tick(s);
            }
        }
        break;
    case R_CR2:   s->cr2 = v; break;
    case R_CR3:   s->cr3 = v; break;
    case R_BRR:   s->brr = v & 0xFFFF; break;
    case R_GTPR:  s->gtpr = v; break;
    case R_RTOR:  s->rtor = v; break;
    case R_RQR:
        if (v & RQR_RXFRQ) {
            s->isr &= ~ISR_RXNE;
        }
        if (v & RQR_TXFRQ) {
            s->isr |= ISR_TXE;
        }
        break;
    case R_ICR:
        s->isr &= ~(v & (ISR_PE | ISR_FE | ISR_NE | ISR_ORE | ISR_IDLE | ISR_TC |
                         ISR_CTSIF | ISR_RTOF | BIT(8) | BIT(12) | BIT(17) |
                         BIT(20)));
        break;
    case R_TDR:
        if (!(s->cr1 & CR1_TE) || s->watch_tag) {
            break;
        }
        s->tdr = v & 0x1FF;
        s->isr &= ~(ISR_TXE | ISR_TC);
        tx_byte(s);
        break;
    case R_PRESC: s->presc = v & 0xF; break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32h7-usart: write 0x%" HWADDR_PRIx "\n",
                      off);
    }
    update(s);
}

static const MemoryRegionOps usart_ops = {
    .read = usart_read,
    .write = usart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void usart_reset(DeviceState *dev)
{
    Stm32h7UsartState *s = STM32H7_USART(dev);

    s->cr1 = s->cr2 = s->cr3 = s->brr = s->gtpr = s->rtor = s->presc = 0;
    s->isr = ISR_TXE | ISR_TC;
    s->rdr = s->tdr = 0;
    s->idle_armed = false;
    fifo8_reset(&s->rxq);
    timer_del(s->rx_timer);
    update(s);
}

static void usart_init(Object *obj)
{
    Stm32h7UsartState *s = STM32H7_USART(obj);

    memory_region_init_io(&s->iomem, obj, &usart_ops, s, "stm32h7-usart", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->dma_rx, "dma-rx", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->dma_tx, "dma-tx", 1);
    s->clk = qdev_init_clock_in(DEVICE(s), "clk", NULL, s, 0);
}

static void usart_realize(DeviceState *dev, Error **errp)
{
    Stm32h7UsartState *s = STM32H7_USART(dev);

    fifo8_create(&s->rxq, RXQ_SIZE);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rx_tick, s);
    qemu_chr_fe_set_handlers(&s->chr, can_receive, receive, NULL, NULL, s, NULL,
                             true);
}

static const Property usart_props[] = {
    DEFINE_PROP_CHR("chardev", Stm32h7UsartState, chr),
};

static void usart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = usart_realize;
    device_class_set_legacy_reset(dc, usart_reset);
    device_class_set_props(dc, usart_props);
}

static const TypeInfo usart_info = {
    .name = TYPE_STM32H7_USART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32h7UsartState),
    .instance_init = usart_init,
    .class_init = usart_class_init,
};

static void usart_register(void)
{
    type_register_static(&usart_info);
}
type_init(usart_register)
