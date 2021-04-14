#!/usr/bin/env python3

import motor
import time
import math

def run():
    m = motor.MotorManager()
    m.set_command_mode(motor.ModeDesired.Position)
    t = time.localtime()
    pos = (t[0] + t[1]/60.0 + t[2]/3600.0)*2*math.pi/12
    m.set_command_position([pos])
    m.write_saved_commands()
    time.sleep(1)
    m.set_command_mode(motor.ModeDesired.Velocity)
    m.set_command_velocity([2*math.pi/(24*3600.0)])
    m.write_saved_commands()

if __name__ == "__main__":
    run()
