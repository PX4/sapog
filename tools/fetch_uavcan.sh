#!/bin/bash

cd $(dirname $(readlink -f $0))/..
git clone https://github.com/pavel-kirienko/uavcan uavcan
