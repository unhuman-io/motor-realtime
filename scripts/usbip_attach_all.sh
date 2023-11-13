#!/bin/bash

set -eo pipefail

if [ $# -ne 1 ]; then
    echo "useage $0 REMOTE_IP"
    exit 1
fi

sudo modprobe usbip-core

devices=$(usbip list -r $1 | grep -Po '(?<= )(\d-[\d\.]+)')

for device in $devices; do
    echo "attaching" $device "from" $1
    sudo usbip attach -r $1 -b $device
done

sleep 1

./set_timeout_all.sh 200