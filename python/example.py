#!/usr/bin/env python3

import motor
import time

m = motor.MotorManager()
motors = m.get_connected_motors()
print(m)
print([m.name() for m in motors])
m

m.set_auto_count()
m.write_saved_commands()
print(m.commands())
s = m.read()[0]
print(s.host_timestamp_received)

print(motors[0]["kp"])
motors[0]["spi"].set("424b")
motors[0]["spi"].get()
motors[0]["spi"].set("1200000000")
s = motors[0]["spi"]
while(1):
    a = str(s)
    b = a[-6:]
    c = int(b,16)
    if c >= 0x800000:
        c -= 0x1000000
    print(c, flush=True)
    time.sleep(.01)