#!/usr/bin/env python3

import motor
import time
import signal
import sys
import re
import argparse



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
encs = ["o"]
for enc in encs:
    if enc == "o":
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

    print("finish {}cal: {}".format(enc, mot[enc+"cal"].get()))

print("finished with {} errors".format(error_count))

stop()
