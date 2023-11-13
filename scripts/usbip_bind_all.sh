#!/bin/bash

sudo modprobe usbip-core usbip-host

paths=$(motor_util --list-path-only)

for path in $paths; do
    sudo usbip bind -b $path
done
