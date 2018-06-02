#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "You are not root. Why $USER, why?!" 1>&2
    exit 1
fi

set -o xtrace

if ! which arm-none-eabi-gdb; then
    # https://bugs.launchpad.net/ubuntu/+source/gdb-arm-none-eabi/+bug/1267680
    apt-get -o Dpkg::Options::="--force-overwrite" install -y gcc-arm-none-eabi gdb-arm-none-eabi
fi

apt-get install -y python3 python3-pip can-utils

pip3 install colorama easywebdav pyserial numpy pyyaml
