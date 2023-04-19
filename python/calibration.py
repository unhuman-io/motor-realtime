#!/usr/bin/env python3

import motor
import pyvisa
import subprocess
import re
import time
import numpy as np
import sys
import argparse

class Logger(object):
    def __init__(self):
        self.terminal = sys.stdout
        self.log = open("log.dat", "a")

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
    
    def flush(self):
        self.log.flush()
        self.terminal.flush()

sys.stdout = Logger()

def program(firmware_script):
        print("Programming " + firmware_script)
        result = subprocess.run([firmware_script], capture_output=True)
        match = re.findall(r"^File downloaded successfully", result.stdout.decode(), re.MULTILINE)
        if len(match) == 2:
            print("Successful download")
            return True
        else:
            print("Download failure")
            return False

def api_tuple_to_numpy(tup):
    return np.asarray(list((float(str(x)) for x in tup)))

class PowerSupplySim():
    def __init__(self):
        pass
    
    def set_voltage(self,voltage):
        pass
        
    def get_voltage(self):
        return 0

    def set_current(self,current):
        pass

    def get_current(self):
        return 0

    def set_on(self):
        pass

    def set_off(self):
        pass

class PowerSupply():
    def __init__(self, instrument, channel=1):
        a = pyvisa.ResourceManager()
        self.inst = instrument
        self.channel = channel
    
    def set_voltage(self,voltage):
        self.inst.write(":sour{}:volt {}".format(self.channel, voltage))
        
    def get_voltage(self):
        return self.inst.query_ascii_values(":meas? ch{}".format(self.channel))[0]

    def set_current(self,current):
        self.inst.write(":sour{}:curr {}".format(self.channel,current))

    def get_current(self):
        return self.inst.query_ascii_values(":meas:curr? ch{}".format(self.channel))[0]

    def set_on(self):
        self.inst.write(":outp ch{},on".format(self.channel))

    def set_off(self):
        self.inst.write(":outp ch{},off".format(self.channel))
    

class Calibration():
    def __init__(self, args):
        self.m = motor.MotorManager()
        #self.motor = self.m.get_connected_motors()[0]
        a = pyvisa.ResourceManager()
        if not args.no_supplies:
            self.inst = a.open_resource('USB0::6833::3601::DP8B235303062::0::INSTR')
            self.inst2 = a.open_resource('USB0::6833::3601::DP8E240900054::0::INSTR')
            self.inst.write("*rst")
            self.inst2.write("*rst")
            self.ps3v3 = PowerSupply(self.inst, 3)
            self.ps5v = PowerSupply(self.inst, 2)
            self.ps48v = PowerSupply(self.inst2, 1)
            self.ps10a = PowerSupply(self.inst2, 2)
        else:
            self.ps3v3 = PowerSupplySim()
            self.ps5v = PowerSupplySim()
            self.ps48v = PowerSupplySim()
            self.ps10a = PowerSupplySim()
        self.args = args


    def run(self):
        #print("Calibrating: " + str(self.motor))
        self.ps3v3.set_voltage(3.3)
        self.ps3v3.set_current(0.1)
        self.ps3v3.set_on()
        time.sleep(1)
        self.measure_3v3_current()
        self.ps3v3.set_voltage(0)
        self.ps5v.set_voltage(5)
        self.ps5v.set_current(0.1)
        self.ps5v.set_on()
        time.sleep(1)
        v5v_current_unprogrammed = self.ps5v.get_current()
        print("5V measured current unprogrammed: {}".format(v5v_current_unprogrammed))
        v3v3 = self.measure_3v3()
        self.ps5v.set_off()
        input("remove 3.3V, hold boot button and press enter")
        self.ps5v.set_on()
        time.sleep(1)
        if not self.args.no_firmware:
            program(args.firmware_script)
        time.sleep(3)
        self.motor = self.m.get_connected_motors()[0]
        i5v_programmed = self.ps5v.get_current()
        print("5V measured current programmed: {}".format(i5v_programmed))
        self.read_sn()
        self.read_3v3()
        self.read_Tboard()
        self.ps48v.set_voltage(48)
        self.ps48v.set_current(0.1)
        self.ps48v.set_on()
        time.sleep(1)
        self.measure_48v()
        self.read_48v()
        self.measure_48v_current()
        time.sleep(1)
        self.m.set_command_mode(motor.ModeDesired.DriverEnable)
        self.m.write_saved_commands()
        time.sleep(1)
        self.measure_48v_current()
        time.sleep(0.1)
        self.m.set_command_mode(motor.ModeDesired.Voltage)
        self.m.write_saved_commands()
        time.sleep(1)
        self.measure_48v_current()
        time.sleep(0.1)
        self.m.set_command_mode(motor.ModeDesired.Damped)
        self.m.write_saved_commands()
        time.sleep(0.1)
        self.ps10a.set_voltage(1)
        self.ps10a.set_current(0)
        self.ps10a.set_on()
        phase_cal_ab = self.measure_phases()
        input("switch phases and hit enter")
        phase_cal_c = self.measure_phases()
        output = """.fast_loop_param.ia_bias = {:.3f},
.fast_loop_param.ib_bias = {:.3f},
.fast_loop_param.ic_bias = {:.3f},
.fast_loop_param.adc1_gain = -3.3/4096/(.001*10)*{:.3f},
.fast_loop_param.adc2_gain = -3.3/4096/(.001*10)*{:.3f},
.fast_loop_param.adc3_gain = -3.3/4096/(.001*10)*{:.3f},
""".format(phase_cal_ab["bias"][0], phase_cal_ab["bias"][1], phase_cal_c["bias"][2],
           abs(phase_cal_ab["gain"][0]), abs(phase_cal_ab["gain"][1]), abs(phase_cal_c["gain"][2]))
        with open(self.motor.serial_number() + ".h", "w") as f:
            f.write(output)

        self.m.set_command_mode(motor.ModeDesired.Sleep)
        self.m.write_saved_commands()
        input("check white led")
        self.inst.write("*rst")
        self.inst2.write("*rst")


    def measure_phases(self):
        self.ps10a.set_current(0)
        time.sleep(1)
        i0 = self.measure_phase_current()
        iabc0 = self.read_phase_current()
        self.ps10a.set_current(2)
        time.sleep(1)
        i2 = self.measure_phase_current()
        iabc2 =self.read_phase_current()
        self.ps10a.set_current(10)
        time.sleep(1)
        i10 = self.measure_phase_current()
        iabc10 = self.read_phase_current()
        self.ps10a.set_current(0)
        print(iabc2 - iabc0)
        print(iabc10 - iabc0)
        gain = (i10-i0)/abs(iabc10 - iabc0)
        print("gain: {}".format(gain))
        iabc_bias = gain*iabc0
        print("bias: {}".format(iabc_bias))
        return {"gain": gain, "bias": iabc_bias}

    def read_Tboard(self):
        Tboard = self.motor["Tboard"]
        print("Tboard read: {}".format(Tboard))
        return Tboard

    def read_3v3(self):
        v3v3 = self.motor["3v3"]
        print("3v3 read: {}".format(v3v3))
        return v3v3
    
    def measure_3v3(self):
        v3v3 = self.ps3v3.get_voltage()
        print("3v3 measured voltage: {}".format(v3v3))
        return v3v3

    def measure_3v3_current(self):
        i3v3 = self.ps3v3.get_current()
        print("3v3 measured current: {}".format(i3v3))
        return i3v3
    
    def measure_48v(self):
        v48v = self.ps48v.get_voltage()
        print("48v measured voltage: {}".format(v48v))
        return v48v
    
    def read_48v(self):
        v48v = self.motor["vbus"]
        print("48v read voltage: {}".format(v48v))
        return v48v

    def measure_48v_current(self):
        i48v = self.ps48v.get_current()
        print("48v measured current: {}".format(i48v))
        return i48v

    def measure_phase_current(self):
        iphase = self.ps10a.get_current()
        print("phase measured current: {}".format(iphase))
        return iphase

    def read_phase_current(self):
        self.motor["zero_current_sensors"] = "5"
        time.sleep(5.1)
        i_abc = list((self.motor["ia_bias"], self.motor["ib_bias"], self.motor["ic_bias"]))
        print("i_abc read: {}, {}, {}".format(*i_abc))
        return api_tuple_to_numpy(i_abc)
    
    def read_sn(self):
        print("serial number: {}".format(self.motor.serial_number()))

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--firmware_script", help="path to firmware programming script", 
                        default="../../obot-controller/obot_g474/build/motor_aksim/load_motor_aksim.sh")
    parser.add_argument("-n", "--no-firmware", help="don't try and program firmware", action="store_true")
    parser.add_argument("--no-supplies", help="run without setting power supplies", action="store_true")
    args = parser.parse_args()
    c = Calibration(args)

    time.sleep(1)
    c.run()