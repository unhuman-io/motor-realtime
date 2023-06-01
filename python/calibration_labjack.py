#!/usr/bin/env python3

import motor
import json
import time
from power_supply import rigol, sim
import firmware_tools
from labjack import ljm # pip install labjack-ljm
import logger

rigol1 = rigol.Instrument('USB0::6833::3601::DP8E240900054::0::INSTR')
ps5V = sim.PowerSupply()
ps48V = rigol.PowerSupply(rigol1)
ps10A = rigol.PowerSupply(rigol1, channel=2)

handle = ljm.openS("ANY", "ANY", "ANY") 
info = ljm.getHandleInfo(handle)
print("Opened a LabJack with Device type: %i, Connection type: %i,\n"
      "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
      (info[0], info[1], info[2], ljm.numberToIP(info[3]), info[4], info[5]))

dio = {"i+_a": "FIO1", "i-_b": "FIO2", "i-_c": "FIO3", "i-_48v": "FIO3"}
aio = {"3V3": "AIN8", "5V": "AIN10", "I5V": "AIN13"}
names = ["AIN13_NEGATIVE_CH", "AIN13_RANGE", "AIN13_RESOLUTION_INDEX", "AIN13_SETTLING_US"]
aValues = [12, 0.01, 0, 0]
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

# measure 5V voltage
v5V_measured = ljm.eReadName(handle, aio["5V"])
assert abs(v5V_measured - 5) < .1
print("5V voltage measured {}".format(v5V_measured))

# measure 5V current
i5V_measured = ljm.eReadName(handle, aio["I5V"])
assert abs(i5V_measured - .12) < .05
print("5V current measured {}".format(i5V_measured))

# measure 3v3 voltage
v3v3_measured = ljm.eReadName(handle, aio["3V3"])
assert abs(v3v3_measured - 3.3) < .1
print("3v3 voltage measured {}".format(v3v3_measured))

# program
# firmware_tools.program()

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
assert abs(v3v3_measured - 3.3) < .1

# motor read 3v3 voltage
v3v3_read = api_average(test_motor["3v3"], 1000)
print("3v3 voltage measured {}, read {}".format(v3v3_measured, v3v3_read))

# calibrate 3v3 voltage
v3v3_calibration = v3v3_measured/v3v3_read
assert abs(v3v3_calibration - 1) < .1
#output_dict["v3v3_calibration"] = "{}".format(v3v3_calibration)

# measure 5V current 
i5V_measured = ljm.eReadName(handle, aio["I5V"])/.01
assert abs(i5V_measured - .12) < .05

# motor read 5V current
i5V_read = api_average(test_motor["i5V"], 1000)
print("5V current measured {}, read {}".format(i5V_measured, i5V_read))

# calibrate 5V current
i5V_calibration = i5V_measured/i5V_read
assert abs(i5V_calibration - 1) < .1
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

# set mode pwm, measure 48V current
m.set_command_mode(motor.ModeDesired.Voltage) # 50% duty cycle voltage
m.write_saved_commands()
time.sleep(1)
i48V = ps48V.get_current()
print("48V current PWM enabled: {}".format(i48V))

# zero current sensors
test_motor["zero_current_sensors"] = "4"
time.sleep(4.1)
def record_current_sensor_bias(name):
    bias = float(test_motor[name + "_bias"].get())
    assert abs(bias) < 10
    output_dict["fast_loop_param"][name + "_bias"] = "{}".format(bias)

record_current_sensor_bias("ia")
record_current_sensor_bias("ib")
record_current_sensor_bias("ic")

# set mode damped and run 10A current through phases
m.set_command_mode(motor.ModeDesired.Damped)
m.write_saved_commands()

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"], dio["i-_48v"]]
aValues = [1, 1, 0, 0]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)
ps10A.set_voltage(3)
ps10A.set_current(10)
ps10A.set_on()

time.sleep(.5)

def record_current_sensor_calibration(name):
    name_map={"ia": "adc1", "ib": "adc2", "ic": "adc3"}
    current = api_average(test_motor[name], 1000)
    print("{} read: {}".format(name, current))
    if name != "i48V":
        assert abs(abs(current) - 10) < 1
        output_dict["fast_loop_param"][name_map[name] + "_gain"] = "-3.3/4096/(.0005*10)*{}".format(10/abs(current))
    else:
        assert abs(abs(current) - 5) < 1

record_current_sensor_calibration("ia")
record_current_sensor_calibration("ib")

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"]]
aValues = [1, 0, 1]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)
time.sleep(.5)

record_current_sensor_calibration("ic")


# set mode damped high
m.set_command_mode(motor.ModeDesired.Voltage) # 50% duty cycle voltage
m.write_saved_commands()

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"], dio["i-_48v"]]
aValues = [1, 0, 0, 1]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)
time.sleep(.5)

record_current_sensor_calibration("i48V")


#rigol1.off()

names = [dio["i+_a"], dio["i-_b"], dio["i-_c"], dio["i-_48v"]]
aValues = [0, 0, 0, 0]
numFrames = len(names)
ljm.eWriteNames(handle, numFrames, names, aValues)

# Close handle
ljm.close(handle)

print(json.dumps(output_dict, indent=2))
with open(serial_number + ".json",'w') as f:
    json.dump(output_dict, f, indent=2)
