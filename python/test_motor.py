#!/usr/bin/env python3

import motor
import time
import unittest
import numpy as np

class TestMotor(unittest.TestCase):
    def setUp(self):
        self.m = motor.MotorManager()
        self.m.set_auto_count()
        self.f = open("output.txt", "a+")

    def tearDown(self):
        self.m.set_command_mode(motor.ModeDesired.Open)
        self.m.write_saved_commands()
        self.f.close()
    
    def test_1basics(self):
        self.m.set_command_mode(motor.ModeDesired.Open)
        self.m.write_saved_commands()
        time.sleep(0.001)
        self.assertEqual(self.m.read()[0].host_timestamp_received, 1)

    def test_velocity_mode(self):
        pos_start = self.m.read()[0].motor_position
        self.m.set_command_mode(motor.ModeDesired.Velocity)
        self.m.set_command_velocity([1])
        self.m.write_saved_commands()
        time.sleep(10)
        self.m.set_command_velocity([1])
        self.m.write_saved_commands()
        pos_end = self.m.read()[0].motor_position
        pos_diff = pos_end-pos_start
        print("pos diff = " + str(pos_diff))
        self.assertTrue(abs(pos_diff - 10) < .1)

    def test_current_bandwidth(self):
        self.m.set_command_mode(motor.ModeDesired.CurrentTuning)
        # todo current tuning mode uses hacked parameters
        self.m.set_command_current([-.3])
        self.m.set_command_reserved([200])
        self.m.write_saved_commands()
        time_start = time.time()
        a = np.array([])
        while(time.time() - time_start < 10):
            a = np.append(a,[self.m.read()[0].iq])

        window = 1000
        skip = 1000
        b = np.sqrt(np.convolve(a**2,np.ones(window)/float(window),'valid'))
        n = len(b)
        i = np.argmax(b[skip:]<.15)
        bw = (skip + i)/float(n) * 200*10
        print("bandwidth = " + str(bw))
        self.f.write("Benchmarkbandwidth " + str(bw) + " 300 Hz\n")
        self.assertTrue(abs(bw - 800) < 300)

        self.m.set_command_mode(motor.ModeDesired.Open)
        self.m.write_saved_commands()

if __name__ == "__main__":
    unittest.main()