# STM32G491 machine (`stm32g491-generic`)

A model of the STMicroelectronics **STM32G491** (Cortex-M4F, 512 KB flash, RM0440)
for running unmodified Zephyr firmware (MCUboot + application) and, above all,
for feeding it **externally driven inputs whose timing against QEMU's virtual
clock is exact**: GPIO levels, ADC voltages and an AC source with an
optocoupler-style output. Nothing is wired on the machine: pin assignments,
analog levels and the oscillator come from the command line.

This is the same fork as the STM32H745 model ([README.stm32h745.md](README.stm32h745.md));
the FDCAN model (SLCAN text frames on a chardev, see its header comment and
[scripts/stm32h745/slcan_hub.py](scripts/stm32h745/slcan_hub.py)), the USART, IWDG,
RNG and I2C models are shared with it.

## Build

```sh
mkdir build && cd build
../configure --target-list=arm-softmmu --disable-docs --disable-tools
ninja
./qemu-system-arm -machine help | grep g491
```

## Run

```sh
qemu-system-arm \
  -M stm32g491-generic,hse-hz=<HSE Hz>,flash-image=bootloader.hex:app.signed.hex \
  -display none -monitor none \
  -serial null -serial null -serial null -serial mon:stdio \
  -chardev socket,id=fdcan2,host=127.0.0.1,port=6210,server=on,wait=off
```

Machine properties (all optional):

| property | meaning |
| --- | --- |
| `hse-hz` | HSE oscillator frequency (default 8000000). The clock tree (PLL, prescalers) is computed from the RCC registers the firmware writes, so SysTick, timers, ADC conversion time and baud rates follow the firmware's configuration. |
| `flash-image` | `:` separated images loaded into the flash at start: `file.hex` (Intel HEX, absolute addresses) or `file.bin@<offset>`. They are loaded once; afterwards the flash lives on its own, so erase/program by the firmware (settings, DFU, MCUboot swap) survive a system reset. `-kernel file.elf` works too. |
| `adc-mv` | initial analog inputs, `<adc>.<channel>=<mV>` separated by `;` (or `,`, which needs `,,` on the `-machine` line), e.g. `adc-mv='2.17=815;2.9=50'` |
| `gpio-in` | initial levels driven onto pins, `<port><pin>=<0\|1>` separated by `;`, e.g. `gpio-in='C0=1;A8=1'` |
| `vref-mv` | ADC reference voltage (VDDA), default 3300 |
| `uid-seed` | varies the 96-bit unique ID, so several instances differ |

Serial ports in `-serial` order: USART1, USART2, USART3, UART4, UART5, LPUART1.
CAN: `-chardev socket,id=fdcan1,...` and/or `id=fdcan2` carry SLCAN frames
(`t<iii><l><data>\r`, `T<iiiiiiii><l><data>\r`). A small client for tests:

```sh
scripts/stm32g491/slcan_tool.py --connect 127.0.0.1:6210 --ext-id 0x1F --data 0000000000000000
```

SRAM is 112 KB at 0x20000000 (SRAM1 + SRAM2 + CCM, the CCM also at 0x10000000).
RAM is not cleared by a system reset (SYSRESETREQ, IWDG), as on the part, and
starts zeroed on a fresh QEMU start, so a retained-RAM region at the top of the
SRAM survives resets. The reset cause is in `RCC_CSR`.

## External inputs

All of these can be changed while the machine runs with QMP (`-qmp
tcp:127.0.0.1:4444,server,nowait`), `qom-set` / `qom-get`.

**GPIO.** Every pin of GPIOA..G has a property `pin0`..`pin15` on
`/machine/soc/gpioa`..`gpiog`: `0` or `1` drives the pin, `-1` releases it (the
pin then follows its pull resistor, or reads 0 when floating). An output pin's
level is the read-only `pad` property (16 bits); `-trace stm32g4_gpio_pad`
logs every pad change. Edges go through EXTI/SYSCFG to the firmware's
interrupts.

```json
{"execute":"qom-set","arguments":{"path":"/machine/soc/gpioc","property":"pin5","value":1}}
{"execute":"qom-get","arguments":{"path":"/machine/soc/gpiob","property":"pad"}}
```

**ADC.** Each channel's input is a voltage in mV, property `adc<N>-in<ch>-mv`
(a number) on `/machine/soc/adc12` (ADC1, ADC2) or `/machine/soc/adc3`:

```json
{"execute":"qom-set","arguments":{"path":"/machine/soc/adc12","property":"adc2-in8-mv","value":1234.5}}
```

The conversion model has the real timing: ADC clock from RCC (kernel clock and
PRESC, or HCLK and CKMODE), per channel sampling time plus 12.5 / 10.5 / 8.5 /
6.5 cycles, software or timer (TRGO) triggers, scan / discontinuous /
continuous modes, DMA requests. The input is **evaluated at the instant the
sampling phase ends**, on the virtual clock.

**AC source** (`-device ac-source`, any number of instances). A sine
`offset + amplitude * sin(2 pi f t + phase)` with an optocoupler-style output
that conducts while `|v| >= threshold`, so it drops out in a window around every
zero crossing (`half-wave` = 1 or 2 keeps only the positive or negative half).

```sh
-device ac-source,id=ac0,freq-mhz=60000,amplitude-mv=24000,threshold-mv=3000,\
gpio=C0,invert=true,adc=1.12,analog-mode=opto,opto-high-mv=3300,opto-low-mv=100,rc-us=20
```

* `gpio=<port><pin>`: drives that GPIO input (high while conducting, low with
  `invert=true`); the edges are scheduled on the virtual clock at the exact
  crossing times (`-trace ac_source_edge` logs them).
* `adc=<adc>.<ch>[,...]`: analog output into ADC channels, evaluated at each
  conversion's sampling instant. `analog-mode=sine` (default): the waveform in mV
  with an optional first order RC low pass (`rc-us`, steady state gain and phase
  lag). `analog-mode=opto`: the node an optocoupler transistor with a pull-up
  drives, `opto-high-mv` while not conducting, `opto-low-mv` while conducting,
  with first order RC edges.
* Runtime (qom-set on `/machine/peripheral/<id>`): `freq-mhz` (milli-Hz; the phase
  stays continuous), `phase-deg` (steps the wave), `amplitude-mv` (brownout),
  `offset-mv`, `threshold-mv`, `enabled`, `half-wave`, `invert`, `rc-us`,
  `opto-high-mv`, `opto-low-mv`.

## Determinism

Everything is driven by QEMU_CLOCK_VIRTUAL, so with `-icount shift=<N>,sleep=off`
(or `shift=auto`) a run is reproducible and runs faster than real time while the
guest idles; without `-icount` the guest's execution speed is that of the host
and the phase between firmware activity and the 60 Hz wave varies from run to run.
`shift=3` is 8 ns per instruction, close to a 150 MHz core.

## What is modeled

Cortex-M4F; flash (dual bank, page/mass erase, double-word programming with the
part's rules, unlock, option byte defaults); RCC (clock tree, ready bits, reset
flags), PWR, SYSCFG (EXTICR), EXTI (GPIO lines), GPIOA..G, USART1..3, UART4..5,
LPUART1 (shared USART model, line-rate reception), FDCAN1/2 (G4 register map and
message RAM, SLCAN bus), ADC1..3, DMA1/2 + DMAMUX, TIM1..4, 6..8, 15..17
(counter, update event, TRGO), IWDG, RNG, I2C1..3, DBGMCU IDCODE, unique ID, flash
size and factory calibration values.

Stubbed (accesses are accepted and logged with `-d unimp`): RTC, TAMP, WWDG,
LPTIM, TIM20, SPI, USB, DAC, comparators, OPAMP, UCPD, SAI, HRTIM, CORDIC, FMAC,
CRC, QUADSPI. Not modeled: injected ADC conversions, analog watchdogs,
oversampling, capture/compare in the timers, low power modes.

## Provenance

The STM32G491 files (`hw/arm/stm32g491_*`, `hw/misc/stm32g4_*`, `hw/misc/ac_source.c`,
`hw/gpio/stm32g4_gpio.c`, `hw/adc/stm32g4_adc.c`, `hw/dma/stm32g4_dma.c`,
`hw/timer/stm32g4_tim.c`, the G4 mode of the FDCAN model, their headers and this
document) were written with an AI coding assistant (Claude) and reviewed by
people. See the note in [README.stm32h745.md](README.stm32h745.md): do not submit
them to upstream QEMU.
