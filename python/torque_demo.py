#!/usr/bin/env python3

import motor
import time



class TorqueDemo:
    torque_value = .3    # Nm output
    velocity_limit = 2  # rad/s output
    wait_time = 1       # seconds
    dt = 0.1           # seconds run frequency
    motor_cpr = 2048*100    # counts per output revolution

    def __init__(self):
        self.m = motor.MotorManager()
        # actuator is set to time out to damped mode unless it receives periodic 
        # commands with different host timestamp. Auto count helps with this
        self.m.set_auto_count()
        self.set_hold_mode()

    def set_torque_mode(self):
        self.state = "torque"
        print("Torque mode")
        self.m.set_command_mode(motor.ModeDesired.Torque)
        self.m.set_command_torque([self.torque_sign * self.torque_value])

    def set_hold_mode(self):
        self.state = "hold"
        print("Hold mode")
        self.m.set_command_mode(motor.ModeDesired.Velocity)

    def set_wait_mode(self):
        self.state = "wait"
        print("Wait mode")
        self.t_start = time.time()
        self.m.set_command_mode(motor.ModeDesired.Velocity)


    def run(self):
        print("Starting torque demo")
        
        status = self.m.read()[0]
        last_encoder = status.motor_encoder
        last_mcu_time = status.mcu_timestamp
        try:
            while(True):
                # read motor
                status = self.m.read()[0]
                velocity = float(motor.diff_encoder(status.motor_encoder, last_encoder))/self.motor_cpr/ \
                    float(motor.diff_mcu_time(status.mcu_timestamp, last_mcu_time))*170e6
                last_mcu_time = status.mcu_timestamp
                last_encoder = status.motor_encoder

                print("mode: {}, torque: {}, velocity: {}".format(self.state, status.torque, velocity))

                # update state
                if (self.state == "hold"):
                    if (abs(status.torque) > self.torque_value):
                        self.torque_sign = 1.0 if status.torque > 0 else -1.0
                        self.set_torque_mode()
                elif (self.state == "torque"):
                    if (abs(velocity) > self.velocity_limit):
                        self.set_wait_mode()
                elif (self.state == "wait"):
                    if (time.time() - self.t_start > self.wait_time):
                        self.set_hold_mode()
                    
                self.m.write_saved_commands()
                time.sleep(self.dt)
        except KeyboardInterrupt:
            print("keyboard interrupt, setting mode open")
            self.m.set_command_mode(motor.ModeDesired.Open)
            self.m.write_saved_commands()

if __name__ == "__main__":
    t = TorqueDemo()
    t.run()