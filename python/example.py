#!/usr/bin/env python3

import motor

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
