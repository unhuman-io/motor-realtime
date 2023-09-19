#!/usr/bin/env python3

import motor
import numpy as np
import time
import signal
import sys
import re



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


print(m)
print(mot.name())

m.set_auto_count()

imax = 6.0
vmax = 20
mac_count = 10
mac_count_ecc = 9


mot["imax"] = str(vmax)
mot["idmax"] = str(vmax)
mot["mac_count"] = str(mac_count)

m.set_command_mode(motor.ModeDesired.Current)
m.set_command_current([imax])
m.write_saved_commands()

print("starting mcal: {}".format(mot["mcal"].get()))

time.sleep(1)

def wait_for_complete(enc = "m", timeout_s=10):
    t_start = time.time()
    while True:
        if (time.time() - t_start > timeout_s):
            print("\ttimeout")
            break
        result_str = mot[enc + "cmd_result"].get()
        match = re.match(r"^command: (\d+), result: (\d+)$", result_str)
        print("\tcommand: {}".format(match[1]))
        if match[1] == "0":
            print("\tresult: {}".format(match[2]))
            break
        time.sleep(.5)

enc = "m"
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
mot["mac_count"] = str(mac_count_ecc)
mot[enc+"auto_ecc"].get()
wait_for_complete(enc,timeout_s=100)

print("finish mcal: {}".format(mot["mcal"].get()))

stop()
