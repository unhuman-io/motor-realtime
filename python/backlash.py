#!/usr/bin/env python3

import motor
import numpy as np
import time
import signal
import sys

n = -50.0


m = motor.MotorManager()
motors = m.get_motors_by_name(["cycloid","resolute"], allow_simulated=True)


def stop():
    print('Stopping')
    m.set_command_mode(motor.ModeDesired.Open)
    m.write_saved_commands()

def signal_handler(sig, frame):
    stop()
    sys.exit(0) 

signal.signal(signal.SIGINT, signal_handler)


positions = np.linspace(0,n*2*np.pi,100)

m.set_command_mode(motor.ModeDesired.Tuning)
c = m.commands()
c[0].tuning_command.tuning_mode = motor.TuningMode.Sine
c[0].tuning_command.mode = motor.ModeDesired.Position
c[0].tuning_command.amplitude = 1
c[0].tuning_command.frequency = 1
m.set_commands(c)
m.write_saved_commands()
tdwell = 2
while(abs(m.read()[0].motor_position) > 1):
    # wait for position
    pass
print("position, backlash")
for position in positions:
    c[0].tuning_command.bias = position
    m.set_commands(c)
    m.write_saved_commands()
    time.sleep(1)
    tstart = time.time()
    error = []
    while(time.time() - tstart < tdwell):
        status = m.read()
        error.append(status[0].motor_position/n - status[1].joint_position)

    backlash = max(error) - min(error)
    print(f'{position}, {backlash}')

stop()
