#!/bin/bash

devices=$(usbip list -r localhost | grep -Po '(?<= )(\d-[\d\.]+)')

for device in $devices; do
    sudo usbip unbind -b $device
done
