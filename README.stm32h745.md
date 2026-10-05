# QEMU with an STM32H745 model

This is QEMU 10.0.2 plus a model of the STMicroelectronics **STM32H745**
microcontroller (Cortex-M7 domain), for running and testing embedded firmware
(for example Zephyr) without hardware. Upstream QEMU has no STM32H7 support.

- Machines: `stm32h745` (core clock set with `sysclk-hz`) and `nucleo-h745zi`
- Models: RCC/PWR/flash interface/HSEM, FDCAN1/2 (with an SLCAN socket bus),
  USART/UART/LPUART, GPIO, the H745 memory map
- Documentation: [docs/system/arm/stm32h745.rst](docs/system/arm/stm32h745.rst)
- CAN bus hub for several nodes: [scripts/stm32h745/slcan_hub.py](scripts/stm32h745/slcan_hub.py)

## Build

```sh
mkdir build && cd build
../configure --target-list=arm-softmmu --disable-docs --disable-tools
ninja
./qemu-system-arm -machine help | grep -E 'stm32h745|nucleo-h745zi'
```

QEMU's usual build dependencies apply (Python 3 with `distlib`, ninja,
glib 2). Add `--disable-pixman` if pixman is not installed.

## Run Zephyr

```sh
west build -b nucleo_h745zi_q/stm32h745xx/m7 zephyr/samples/hello_world
build-qemu/qemu-system-arm -machine nucleo-h745zi -nographic \
    -kernel build/zephyr/zephyr.elf -serial null -serial null -serial mon:stdio
```

## Provenance and upstreaming

The STM32H745 models (`hw/arm/stm32h745_*`, `hw/misc/stm32h7_sysctrl.c`,
`hw/net/can/stm32h7_fdcan.c`, their headers, the documentation and the hub
script) were written with an AI coding assistant (Claude) and reviewed by
people. QEMU's [code provenance policy](https://www.qemu.org/docs/master/devel/code-provenance.html)
currently declines contributions that include or derive from AI-generated
content, so **do not submit these files to upstream QEMU**. An upstream
STM32H7 model would need an independent implementation from the reference
manual (RM0399).

## License

QEMU is licensed under the GPL version 2 (see [COPYING](COPYING)); the
STM32H745 files are GPL-2.0-or-later.
