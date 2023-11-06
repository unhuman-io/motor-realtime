#!/usr/bin/env python3

import motor
import time
import signal
import sys
import re
import argparse
import math
import numpy as np
import matplotlib.pyplot as plt


m = motor.MotorManager()
motors = m.get_connected_motors()
mot = motors[0]

def stop():
    print('Stopping')
    m.set_command_mode(motor.ModeDesired.Open)
    m.write_saved_commands()

def signal_handler(sig, frame):
    stop()
    sys.exit(0) 

signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser()
parser.add_argument("--current", default=6, type=float)
parser.add_argument("--voltage", default=20, type=float)
parser.add_argument("--velocity", default=400, type=float)

args = parser.parse_args()


print(m)
print(mot.name())

m.set_auto_count()

imax = args.current
vmax = args.voltage
velocity = args.velocity
mac_count = 12
mac_count_ecc = 9
oac_count = 6
oac_count_ecc = 3


mot["imax"] = str(vmax)
mot["idmax"] = str(vmax)
mot["mac_count"] = str(mac_count)
mot["oac_count"] = str(oac_count)
mot["olow"] = "1"

m.set_command_mode(motor.ModeDesired.Current)
m.set_command_current([imax])
m.write_saved_commands()



time.sleep(1)

error_count = 0
def wait_for_complete(enc = "m", timeout_s=10):
    global error_count
    t_start = time.time()
    while True:
        if (time.time() - t_start > timeout_s):
            print("\ttimeout")
            error_count += 1
            break
        result_str = mot[enc + "cmd_result"].get()
        match = re.match(r"^command: (\d+), result: (\d+)$", result_str)
        print("\tcommand: {}".format(match[1]))
        if match[1] == "0" or match[1] == '255':
            print("\tresult: {}".format(match[2]))
            if (match[1] == '255'):
                error_count += 1
            break
        time.sleep(.5)

encs = ["m", "o"]
#encs = ["m"]
for enc in encs:
    if enc == "o":
        mot["mecc_correction"] = "1"
        m.set_command_mode(motor.ModeDesired.Open)
        m.write_saved_commands()
        time.sleep(1)
        m.set_command_mode(motor.ModeDesired.Velocity)
        m.set_command_current([0])
        m.set_command_velocity([velocity])
        m.write_saved_commands()
        time.sleep(1)

    print("starting {}cal: {}".format(enc, mot[enc+"cal"].get()))
    mot[enc+"ecc_correction"] = "0"
    print(enc+"auto_ana")
    mot[enc+"auto_ana"].get()
    wait_for_complete(enc)

    print(enc+"auto_dig")
    mot[enc+"auto_dig"].get()
    wait_for_complete(enc)

    print(enc+"readj_dig")
    mot[enc+"readj_dig"].get()
    wait_for_complete(enc)

    print(enc+"auto_ecc")
    mot[enc+"ac_count"] = str(eval(enc+"ac_count_ecc"))
    mot[enc+"auto_ecc"].get()
    wait_for_complete(enc,timeout_s=20)

    if enc == "o":
        # encoder vs encoder eccentricity
        max_diff = float("-inf")
        min_diff = float("inf")
        status = m.read_average(1)[0]
        joint_start = status.joint_position
        motor_start = status.motor_position
        motor_last = motor_start
        joint_position = joint_start
        gear_ratio = float(mot["gear_ratio"].get())
        mrollover = float(mot["mrollover"].get())
        disk_um = float(mot["odisk_um"].get())

        while(abs(joint_position - joint_start) < 3*math.pi):
            status = m.read_average(1)[0]
            joint_position = status.joint_position
            motor_position = np.unwrap([motor_last,status.motor_position], period=mrollover)[1]
            motor_last = motor_position
            diff = status.joint_position -joint_start - (motor_position-motor_start)/gear_ratio
            #print(f'diff: {diff}')
            if diff > max_diff:
                max_diff = diff
            if diff < min_diff:
                min_diff = diff
        print(f"encoder peak-peak diff um: {(max_diff - min_diff)*disk_um}")
    
    if enc == "m":
        # encoder vs const slope eccentricity
        status = m.read_average(1)[0]
        motor_start = status.motor_position
        motor_last = motor_start
        disk_um = float(mot["mdisk_um"].get())
        mrollover = float(mot["mrollover"].get())
        motor_position = motor_start
        t_last = status.mcu_timestamp
        t = []
        mp = []

        while(abs(motor_position - motor_start) < 300*2*math.pi):
            status = m.read_average(1)[0]
            motor_position = np.unwrap([motor_last,status.motor_position], period=mrollover)[1]
            motor_last = motor_position
            t_s = np.unwrap([t_last, status.mcu_timestamp], period=2**32)[1] / 170e6
            t_last = status.mcu_timestamp
            t.append(t_s)
            mp.append(motor_position - motor_start)

        t = np.array(t)
        mp = np.array(mp)
        print(t, mp)
        vel = (mp[-1] - mp[0])/(t[-1] - t[0])
        print(f"vel avg: {vel}")
        mv = vel * (t - t[0]) + mp[0]

        stop()
        plt.plot(mp % (2*np.pi),(mp-mv)*disk_um)
        error = (mp-mv)

        
        rev = np.floor(mp / 2*np.pi)
        avg = np.zeros(int(rev[-1]))
        error_unbias = np.zeros(np.size(error))
        for i in range(int(rev[-1])):
            avg[i] = np.average(error[rev == i])
            error_unbias[rev == i] = error[rev == i] - avg[i]

        # bins = np.linspace(0, 2*np.pi, 1001)
        # inds = np.digitize(mp % (2*np.pi), bins)
        # count = np.zeros(1001)
        # avg = np.zeros(1001)
        # for i in range(1001):
        #     count[i] = np.sum(i == inds)
        #     avg[i] = np.average(error[inds == i])

        plt.figure(2)
        plt.plot(mp, error_unbias,'.')
        plt.show()

    print("finish {}cal: {}".format(enc, mot[enc+"cal"].get()))

print("finished with {} errors".format(error_count))

stop()
