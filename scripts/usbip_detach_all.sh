#!/bin/bash

devices=$(usbip port | grep -Po '(?<=Port )(\d+)')

for device in $devices; do
    echo "detaching" $device
    sudo usbip detach -p $device
done
