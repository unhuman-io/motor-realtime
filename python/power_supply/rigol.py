import pyvisa
import time

class Instrument():
    inst = None

    def __init__(self, name='USB0::6833::3601::DP8B235303062::0::INSTR'):
        a = pyvisa.ResourceManager()
        try:
            self.inst = a.open_resource(name)
        except ValueError:
            print("Error connecting Rigol: {}".format(name))
            exit(1)
        self.inst.write("*rst")

    def off(self):
        print('Rigol set off')
        if self.inst:
            self.inst.write("*rst")

class PowerSupply():
    def __init__(self, instrument=None, channel=1):
        if not instrument:
            instrument=Instrument()
        self.inst = instrument.inst
        self.channel = channel
    
    def set_voltage(self,voltage):
        self.inst.write(":sour{}:volt {}".format(self.channel, voltage))
        
    def get_voltage(self):
        return self.inst.query_ascii_values(":meas? ch{}".format(self.channel))[0]

    def set_current(self,current):
        self.inst.write(":sour{}:curr {}".format(self.channel,current))

    def get_current(self):
        # a = self.inst.query_ascii_values(":meas:curr? ch{}".format(self.channel))[0]
        # print(a)
        # time.sleep(1)
        return self.inst.query_ascii_values(":meas:curr? ch{}".format(self.channel))[0]

    def set_on(self):
        self.inst.write(":outp ch{},on".format(self.channel))

    def set_off(self):
        self.inst.write(":outp ch{},off".format(self.channel))