#!/usr/bin/env python3

import motor
import time
import unittest
import sys
import numpy as np

path = None
class TestMotor(unittest.TestCase):
    m = None
    f = None

    @classmethod
    def setUpClass(cls):
        cls.m = motor.MotorManager()
        if (path):
            cls.m.get_motors_by_path([path])
        cls.m.set_auto_count()
        cls.f = open("output.txt", "a+")

    @classmethod
    def tearDownClass(cls):
        cls.m.set_command_mode(motor.ModeDesired.Open)
        cls.m.write_saved_commands()
        cls.f.close()

    def tearDown(self):
        self.m.set_commands([motor.Command()])
    
    def test_1basics(self):
        self.m.set_command_mode(motor.ModeDesired.Open)
        self.m.write_saved_commands()
        time.sleep(0.001)
        self.assertEqual(self.m.read()[0].host_timestamp_received, 1)

    def test_velocity_mode(self):
        t = 10.0
        v = 5.0
        n = 5.4
        status_start = self.m.read()[0]
        pos_start = status_start.motor_position
        output_start = status_start.joint_position
        self.m.set_command_mode(motor.ModeDesired.Velocity)
        self.m.set_command_velocity([v])
        self.m.write_saved_commands()
        time.sleep(t)
        self.m.set_command_velocity([1])
        self.m.write_saved_commands()
        status_end = self.m.read()[0]
        pos_end = status_end.motor_position
        pos_diff = pos_end-pos_start
        output_diff = status_end.joint_position - output_start
        print("pos diff = " + str(pos_diff))
        print("output_diff = " + str(output_diff*n))
        self.assertTrue(abs(pos_diff - t*v) < .1)
        self.assertTrue(abs(output_diff*n - t*v) < 0.16*n)

    def test_current_bandwidth(self):
        self.m.set_command_current_tuning(motor.TuningMode.Chirp, .3, 200, 0)
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
        self.f.write("Benchmarkbandwidth 300 " + str(bw) + " Hz\n")
        self.assertTrue(abs(bw - 800) < 300)

        self.m.set_command_mode(motor.ModeDesired.Open)
        self.m.write_saved_commands()

if __name__ == "__main__":
    if(len(sys.argv) > 1 and sys.argv[1] == '-p'):
        path = sys.argv[2]
        del(sys.argv[1:3])
    unittest.main()