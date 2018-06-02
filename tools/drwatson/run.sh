#!/bin/bash
set -e
git pull > /dev/null
git submodule update --init --recursive
sudo ./drwatson_sapog.py $@
