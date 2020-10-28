#!/usr/bin/env python3

import motor

m = motor.MotorManager()
motors = m.get_connected_motors()
print(m)
print([m.name() for m in motors])
m