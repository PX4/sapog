PX4ESC firmware
===============

Under construction.

### Build instructions
Compiler: GCC ARM 4.6+
```bash
cd tools
./fetch_chibios.sh
./fetch_libcanaerospace.sh # Or make a symlink instead
cd ..
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```
Execute `./blackmagic_flash.sh [portname]` from the `tools` directory to flash the firmware with a Black Magic Debug Probe.
