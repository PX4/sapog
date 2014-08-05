#!/bin/bash

cd $(dirname $(readlink -f $0))/../firmware
git clone https://github.com/ChibiOS-Upstream/ChibiOS-RT.git chibios
cd chibios
#git checkout stable_2.6.x
# ICU doesn't work at b74842655f733deb5808372e6d437a4ecfe7ad73
git checkout f19ef0308363fd4f2d76eaac6f6dd0e1e500023b
