PX4 Sapog
=========

Please refer to the documentation page at <https://pixhawk.org/firmware/sapog>.

### Hardware timer usage

* TIM1 - 3-phase FET bridge PWM
* TIM2 - ADC synchronization, works in lockstep with TIM1
* TIM3 - RGB LED PWM
* TIM4 - Hard real time callout interface for motor control logic (preempts the kernel)
* TIM5 - RC PWM input capture
* TIM6 - High precision timestamping for motor control logic (sub-microsecond resolution, never overflows)
* TIM7 - General purpose timestamping

### Build instructions

Prerequisites:

* GCC ARM 4.7+
* Python 2.7 or Python 3.2+

#### Firmware

```bash
git submodule update --init --recursive
cd firmware
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```

Execute `./blackmagic_flash.sh [portname]` from the `tools` directory to flash the firmware with a Black Magic Debug Probe.

#### [UAVCAN](http://uavcan.org/) testing tool

This step is optional. Works only for Linux.

Make sure the libuavcan is installed in the system. If not yet:

```bash
cd firmware/uavcan
mkdir build
cd build
cmake ..
make
sudo make install
```

Build the UAVCAN testing tool:
```bash
cd tools/uavcan_tool
mkdir build
cd build
cmake ..
make
```
