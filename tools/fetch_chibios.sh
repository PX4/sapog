#!/bin/bash

cd $(dirname $(readlink -f $0))/../firmware
git clone https://github.com/ChibiOS-Upstream/ChibiOS-RT.git chibios
cd chibios
git checkout stable_2.6.x
