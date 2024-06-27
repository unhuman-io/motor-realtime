#!/bin/bash -e

can=can0
addr=c310000
sudo ip link set down $can
echo -n 10 | sudo tee /sys/module/mttcan/drivers/platform\:mttcan/$addr.mttcan/net/$can/tdc_offset
sudo ip link set $can type can bitrate 1000000 dbitrate 5000000 fd on one-shot on
sudo ip link set up $can
