#!/bin/bash

serials=$(dfu-util -l | grep "@Internal Flash" | sed -nr 's/.*serial="([^"]*)"/\1/p')

for serial in $serials; do
    rm -f tmp1.dat
    dfu-util -a0 -s 0x8060000:leave -U tmp1.dat -S $serial
    rm tmp1.dat
done
