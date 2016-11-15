PX4 Sapog
=========

[![Join the chat at https://gitter.im/Zubax/general](https://img.shields.io/badge/GITTER-join%20chat-green.svg)](https://gitter.im/Zubax/general)

**Please refer to the documentation page at <https://docs.zubax.com/sapog>.**

## Firmware

If you're not running Linux or OSX natively, you can download
[Bistromathic - a Linux virtual machine pre-configured for embedded development](https://files.zubax.com/vm/bistromathic.ova).

### Bootloader

The bootloader allows to update the firmware via the standard UAVCAN firmware upgrade protocol,
which is documented at [uavcan.org](http://uavcan.org/Specification/6._Application_level_functions/#firmware-update).
No additional steps are needed to build the bootloader - the build system will build it automatically together with
the firmware. The resulting `*.elf` file will be extended with the bootloader too, so it can be flashed directly into an
factory fresh MCU.

### Build instructions

**Prebuilt binaries are available at <https://files.zubax.com/products/io.px4.sapog/>.**

Prerequisites:

* [GCC ARM 4.9 or newer](https://launchpad.net/gcc-arm-embedded) (beware that *some* newer versions of GCC segfault during linking)
* Python 3.2+
* Linux or OSX host computer (Windows is not supported)

```bash
git submodule update --init --recursive
cd firmware
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```

The build outputs will be stored into `build/`:

* `*.application.bin` - built application binary, suitable for uploading via the bootloader;
* `*.compound.bin` - application binary together with the bootloader, in one image;
* `compound.elf` - application ELF together with the bootloader, in one file; this option is recommended for debugging.

Execute `./blackmagic_flash.sh [portname]` from the `tools` directory to flash the firmware with a Black Magic Debug Probe.

### Development

We recommend Eclipse for IDE, but any other IDE will work equally well.
If you prefer Eclipse and need GUI debugging, avoid upgrading to any version newer than Luna,
since in newer releases GUI GDB debugging of embedded targets is broken.
Otherwise we recommend to use the latest Eclipse together with CLI GDB client.
It's inconvenient, but unlike Eclipse it works reliably.

When editing code, please follow the
[PX4 coding conventions](https://github.com/PX4/Firmware/blob/master/CONTRIBUTING.md).

### Hardware timer usage

* TIM1 - 3-phase FET bridge PWM
* TIM2 - ADC synchronization, works in lockstep with TIM1
* TIM3 - RGB LED PWM
* TIM4 - Hard real time callout interface for motor control logic (preempts the kernel)
* TIM5 - RC PWM input capture
* TIM6 - High precision timestamping for motor control logic (sub-microsecond resolution, never overflows)
* TIM7 - General purpose timestamping

## Hardware

Reference hardware design is published under CC BY-SA 3.0 in the [PX4 Hardware repository](https://github.com/PX4/Hardware).

Known commercially available compatible hardware designs are listed below.

- [Zubax Orel 20](https://docs.zubax.com/zubax_orel_20)
