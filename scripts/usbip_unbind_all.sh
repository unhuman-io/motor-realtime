#!/bin/bash

devices=$(usbip list -r localhost | grep -o " [0-9]-[0-9\.]*")

for device in $devices; do
    sudo usbip unbind -b $device
done
