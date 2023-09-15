#!/usr/bin/env python3

import motor
import numpy as np
import time
import signal
import sys



m = motor.MotorManager()
motors = m.get_motors_by_name(["a1","a2"], allow_simulated=True)
a1 = motors[0]
a2 = motors[1]

def stop():
    print('Stopping')
    m.set_command_mode(motor.ModeDesired.Open)
    m.write_saved_commands()

def signal_handler(sig, frame):
    stop()
    sys.exit(0) 

signal.signal(signal.SIGINT, signal_handler)


print(m)
print([m.name() for m in motors])
m

m.set_auto_count()

imax = 40
vmax = 600
ni = 10
nv = 11
tdwell = .25
accel = 1000

tmp = np.linspace(-imax, imax, ni)
i = np.argsort(abs(tmp))
igrid = tmp[i]
tmp = np.linspace(-vmax, vmax, nv)
i = np.argsort(abs(tmp))
vgrid = tmp[i]


for mot in motors:
    mot["imax"] = "40"
    mot["idmax"] = "40"
    mot["vmax"] = "80"
    mot["vacceleration_limit"] = str(accel)

vlast = 0
for ig in igrid:
    for vg in vgrid:
        print("testing {} A, {} rad/s".format(ig, vg))
        m.set_command_current([0, ig])
        m.set_command_velocity([vg, 0])
        m.set_command_mode([motor.ModeDesired.Velocity, motor.ModeDesired.Current])
        m.write_saved_commands()
        taccel = abs(vg - vlast)/accel + .25
        vlast = vg
        time.sleep(taccel)
        # collect data
        tstart = time.time()
        vel = []
        i = []
        vel2 = []
        i2 = []
        while(time.time() - tstart < tdwell):
            pass
            s = m.read()
            #print(s[0].motor_velocity)
            #print(s[0].iq)
            # print(s[1].motor_velocity)
            # print(s[1].iq)
            # print(s[0])
            vel.append(s[0].motor_velocity)
            i.append(s[0].iq)
            vel2.append(s[1].motor_velocity)
            i2.append(s[1].iq)

        print(np.mean(i), np.mean(vel), np.mean(i2), np.mean(vel2))

stop()
