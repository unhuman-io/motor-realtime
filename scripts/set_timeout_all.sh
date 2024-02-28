#!/bin/bash

if [ $# -ne 1 ]; then
    echo "useage $0 TIMEOUT_MS"
    exit 1
fi

paths=$(motor_util -c none --list-path-only)

for path in $paths; do
    echo "echo $1 > /sys/bus/usb/drivers/usb_rt/$path:1.0/timeout_ms"
    echo $1 > /sys/bus/usb/drivers/usb_rt/$path:1.0/timeout_ms
done

echo "motor_util --set-api api_timeout=${1}000"
motor_util --set-api api_timeout=${1}000
motor_util --set-api api_timeout=${1}000
motor_util --set-api api_timeout=${1}000
