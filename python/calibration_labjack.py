#!/usr/bin/env python3

import motor
import json
import time
from power_supply import rigol, sim
import firmware_tools
from labjack import ljm # pip install labjack-ljm
import logger
import argparse

parser = argparse.ArgumentParser()
# parser.add_argument("-f", "--firmware_script", help="path to firmware programming script", 
#                     default="../../obot-controller/obot_g474/build/motor_aksim/load_motor_aksim.sh")
parser.add_argument("-f", "--firmware", help="program firmware", action="store_true")
parser.add_argument("-p", "--programmed", help="note that the board has already been programmed", action="store_true")
# parser.add_argument("--no-supplies", help="run without setting power supplies", action="store_true")
args = parser.parse_args()

rigol1 = rigol.Instrument('USB0::6833::3601::DP8E240900054::0::INSTR')
rigol0 = rigol.Instrument('USB0::6833::3601::DP8B241100662::0::INSTR')
ps5V = rigol.PowerSupply(rigol0)
ps48V = rigol.PowerSupply(rigol1)
ps10A = rigol.PowerSupply(rigol1, channel=2)

handle = ljm.openS("ANY", "ANY", "ANY") 
info = ljm.getHandleInfo(handle)
print("Opened a LabJack with Device type: %i, Connection type: %i,\n"
      "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
      (info[0], info[1], info[2], ljm.numberToIP(info[3]), info[4], info[5]))

dio = {"prog": "FIO0", "i+_a": "FIO1", "i-_b": "FIO2", "i-_c": "FIO3", "i-_48v": "FIO3"}
aio = {"3V3": "AIN8", "5V": "AIN10", "I5V": "AIN12"}
names = ["AIN12_NEGATIVE_CH", "AIN12_RANGE", "AIN12_RESOLUTION_INDEX", "AIN12_SETTLING_US",
    "AIN13_NEGATIVE_CH", "AIN13_RANGE", "AIN13_RESOLUTION_INDEX", "AIN13_SETTLING_US"]
aValues = [13, 0.01, 0, 0, 199, 0.01, 0, 0]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)


for name in dio.values():
    state = 0  # Output state = low (0 = low, 1 = high)
    ljm.eWriteName(handle, name, state)
   # print("\nSet %s state : %f" % (name, state))

for name in aio.values():
    result = ljm.eReadName(handle, name)
   # print("\n%s reading : %f V" % (name, result))

output_dict = {"fast_loop_param": {}}

ps5V.set_voltage(5)
ps5V.set_current(.2)
ps5V.set_on()


def api_average(api_item, num):
    val = 0.0
    for i in range(num):
        val = val + float(api_item.get())/num
    time.sleep(0.001)
    return val

# actual testing
time.sleep(.5)
# measure 5V voltage
v5V_measured = ljm.eReadName(handle, aio["5V"])
print("5V voltage measured {}".format(v5V_measured))
assert abs(v5V_measured - 5) < .1


# measure 5V current
i5V_measured = ljm.eReadName(handle, aio["I5V"])*(-100)
print("5V current measured {}".format(i5V_measured))
if args.programmed:
    assert abs(i5V_measured - .07) < .02
else:
    assert abs(i5V_measured - .02) < .02

# measure 3v3 voltage
v3v3_measured = ljm.eReadName(handle, aio["3V3"])
print("3v3 voltage measured {}".format(v3v3_measured))
#assert abs(v3v3_measured - 3.3) < .1


if args.firmware:
    #input("power off 5V")
    ps5V.set_off()
    time.sleep(.5)
    ljm.eWriteName(handle, dio["prog"], 1)

    #input("power on 5V")
    ps5V.set_on()
    time.sleep(5)
    # program
    ljm.eWriteName(handle, dio["prog"], 0)
    firmware_tools.program()

time.sleep(3)
# connect to motor
m = motor.MotorManager()
test_motor = m.motors()[0]
serial_number = test_motor.serial_number()
print("Connected to {}".format(serial_number))

# measure 5V voltage 
v5V_measured = ljm.eReadName(handle, aio["5V"])
assert abs(v5V_measured - 5) < .1

# motor read 5V voltage
v5V_read = api_average(test_motor["5V"], 1000)
print("5V voltage measured {}, read {}".format(v5V_measured, v5V_read))

# calibrate 5V voltage
v5V_calibration = v5V_measured/v5V_read
assert abs(v5V_calibration - 1) < .1
#output_dict["v5V_calibration"] = "{}".format(v5V_calibration)

# measure 3v3 voltage
v3v3_measured = ljm.eReadName(handle, aio["3V3"])
#assert abs(v3v3_measured - 3.3) < .1

# motor read 3v3 voltage
v3v3_read = api_average(test_motor["3v3"], 1000)
print("3v3 voltage measured {}, read {}".format(v3v3_measured, v3v3_read))

# calibrate 3v3 voltage
v3v3_calibration = v3v3_measured/v3v3_read
#assert abs(v3v3_calibration - 1) < .1
#output_dict["v3v3_calibration"] = "{}".format(v3v3_calibration)

# measure 5V current 
i5V_measured = -ljm.eReadName(handle, aio["I5V"])/.01


# motor read 5V current
i5V_read = api_average(test_motor["i5V"], 1000)
print("5V current measured {}, read {}".format(i5V_measured, i5V_read))
assert abs(i5V_measured - .05) < .02

# read temperatures
Tboard = api_average(test_motor["Tboard"], 1000)
Tbridge = api_average(test_motor["Tbridge"], 1000)
Tbridge2 = api_average(test_motor["Tbridge2"], 1000)
print("Temperatures Tboard: {}, Tbridge: {}, Tbridge2: {}".format(Tboard, Tbridge, Tbridge2))
assert abs(Tboard - 30) < 20
assert abs(Tboard - Tbridge) < 3
assert abs(Tboard - Tbridge) < 3


# calibrate 5V current
#i5V_calibration = i5V_measured/i5V_read
#assert abs(i5V_calibration - 1) < .1
#output_dict["i5V_gain"] = "1000 * {}".format(i5V_calibration)

ps48V.set_voltage(48)
ps48V.set_current(.2)
ps48V.set_on()

# measure 48V current, set bias
m.set_command_mode(motor.ModeDesired.DriverDisable) # 50% duty cycle voltage
m.write_saved_commands()
time.sleep(1)
i48V = ps48V.get_current()
print("48V current quiescent: {}".format(i48V))

# enable driver, measure 48V current
test_motor["error_mask"] = "0"
m.set_command_mode(motor.ModeDesired.DriverEnable) # 50% duty cycle voltage
m.write_saved_commands()
time.sleep(1)
i48V = ps48V.get_current()
print("48V current driver enabled: {}".format(i48V))

# check driver enable success
s = test_motor["log"].get()
while not "drv8323s" in s:
    time.sleep(0.001)
    s = test_motor["log"].get()
print(s)
assert s.endswith("drv8323s configure success")

# set mode pwm, measure 48V current
m.set_command_mode(motor.ModeDesired.Voltage) # 50% duty cycle voltage
m.write_saved_commands()
time.sleep(1)
i48V = ps48V.get_current()
print("48V current PWM enabled: {}".format(i48V))

# zero current sensors
test_motor["zero_current_sensors"] = "8"
time.sleep(8.5)
def record_current_sensor_bias(name):
    bias = api_average(test_motor[name+"_bias"], 1000)
    print("{} bias: {:.3f}".format(name, bias))
    assert abs(bias) < 10
    output_dict["fast_loop_param"][name + "_bias"] = "{:.3f}".format(bias)

record_current_sensor_bias("ia")
record_current_sensor_bias("ib")
record_current_sensor_bias("ic")

time.sleep(.1)
# set mode damped and run 10A current through phases
m.set_command_mode(motor.ModeDesired.Open)
m.write_saved_commands()
time.sleep(.1)
m.set_command_mode(motor.ModeDesired.Damped)
m.write_saved_commands()
time.sleep(.1)
m.set_command_mode(motor.ModeDesired.Open)
m.write_saved_commands()
time.sleep(.1)
m.set_command_mode(motor.ModeDesired.Damped)
m.write_saved_commands()

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"], dio["i-_48v"]]
aValues = [1, 1, 0, 0]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)
ps10A.set_voltage(3)
ps10A.set_current(10)
ps10A.set_on()

time.sleep(2)
#input("enter to continue")

def record_current_sensor_calibration(name):
    name_map={"ia": "adc1", "ib": "adc2", "ic": "adc3"}
    current = api_average(test_motor[name], 10000)
    print("{} read: {}".format(name, current))
    if name != "i48V":
        assert abs(abs(current) - 10) < .3
        output_dict["fast_loop_param"][name_map[name] + "_gain"] = "-3.3/4096/(.0005*10)*{:.3f}".format(10/abs(current))
        output_dict["fast_loop_param"][name+"_bias"] = output_dict["fast_loop_param"][name+"_bias"] + "*{:.3f}".format(10/abs(current))
    else:
        assert abs(abs(current) - 5) < 1

record_current_sensor_calibration("ia")
record_current_sensor_calibration("ib")

ps10A.set_off()
time.sleep(.5)

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"]]
aValues = [1, 0, 1]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)

ps10A.set_on()
time.sleep(2)
#input("enter to continue")

record_current_sensor_calibration("ic")


# set mode damped high
# m.set_command_mode(motor.ModeDesired.Voltage) # 50% duty cycle voltage
# m.write_saved_commands()

# names = [dio["i+_a"], dio["i-_b"], dio["i-_c"], dio["i-_48v"]]
# aValues = [1, 0, 0, 1]
# numFrames = len(names)
# ljm.eWriteNames(handle, numFrames, names, aValues)
# time.sleep(.5)

#record_current_sensor_calibration("i48V")
m.set_command_mode(motor.ModeDesired.Sleep) # 50% duty cycle voltage
m.write_saved_commands()
input("check LEDs")

rigol1.off()
rigol0.off()

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"], dio["i-_48v"]]
aValues = [0, 0, 0, 0]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)

# Close handle
ljm.close(handle)

print(json.dumps(output_dict, indent=4))
with open(serial_number + ".json",'w') as f:
    json.dump(output_dict, f, indent=4)


print("sn: {} ok!".format(serial_number))
