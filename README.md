PX4 Sapog
=========

[![Join the chat at https://gitter.im/Zubax/general](https://img.shields.io/badge/GITTER-join%20chat-green.svg)](https://gitter.im/Zubax/general)

## Documentation

**[Sapog v2 Reference Manual](https://files.zubax.com/products/io.px4.sapog/Sapog_v2_Reference_Manual.pdf)**
is the main piece of project documentation; make sure to read it.
Additional documentation and related resources can be found at the
**[Zubax Knowledge Base](https://kb.zubax.com/x/cYAh)**.

## Change Log

### v2.3

* Configuration parameter `mot_i_shunt_mr` has been removed; now the firmware detects the shunt resistance
based on the HW ID pin states.
* Migrated from GCC 4.9 to GCC 7.2.

### v2.2

* Added a low-pass filter on the estimated angular speed output.
This change led to improved performance on motors with severe phase asummetry.
* Improved RGB LED indication in the bootloader.

### v2.1

* Fixed stability issues at extremely high RPM (>10k mechanical RPM for 14 pole motor).
* Default PWM frequency set to 60 kHz.

### v2.0

* Completely new, more robust spin up algorithm. Supports smooth start-up from standstill as well as picking up
the rotation if the rotor is already spinning.
* Significantly more reliable operation during rapid acceleration and deceleration,
especially at high advance angles.
* Wider PWM frequency range: 20...75 kHz.
* Raised the maximum RPM limit; the new maximum for 14 pole motor exceeds 14000 mechanical RPM.

## Firmware

If you're not running Linux or OSX natively, you can use
[**Bistromathic** - a Linux virtual machine pre-configured for embedded development](https://kb.zubax.com/x/KIEh).

### Bootloader

The bootloader allows to update the firmware via the standard UAVCAN firmware upgrade protocol,
which is documented at [uavcan.org](http://uavcan.org/Specification/6._Application_level_functions/#firmware-update).
No additional steps are needed to build the bootloader - the build system will build it automatically together with
the firmware. The resulting `*.elf` file will be extended with the bootloader too, so it can be flashed directly into an
factory fresh MCU.

### Build Instructions

**Prebuilt binaries are available at <https://files.zubax.com/products/io.px4.sapog/>.**

Prerequisites:

* GCC ARM 7.2
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

### Hardware Timer Usage

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

- [Zubax Orel 20](https://zubax.com/products/orel_20)
- [Holybro Kotleta20](https://shop.holybro.com/kotleta20_p1156.html)
