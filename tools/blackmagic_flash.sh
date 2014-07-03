#!/bin/bash

PORT=${1:-'/dev/ttyACM0'}
#/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_DDE578CC-if00

# Go to the build outputs directory
cd $(dirname $(readlink -f $0))/../firmware/build

# Find the firmware ELF
elf=$(ls -1 *.elf)
if [ -z "$elf" ]; then
    echo "No firmware found"
    exit 1
fi

arm-none-eabi-size $elf || exit 1

tmpfile=fwupload.tempfile
cat > $tmpfile <<EOF
target extended-remote $PORT
mon swdp_scan
attach 1
load
kill
EOF

arm-none-eabi-gdb $elf --batch -x $tmpfile
rm $tmpfile
