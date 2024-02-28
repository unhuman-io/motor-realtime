#!/usr/bin/env python3

import motor
import time

#some music

m = motor.MotorManager()
motors = m.get_connected_motors()
mot = motors[0]

music = "GFG BAB | gfg gab | GFG BAB"
duration = .1
freq_map = {'G': 392, 'F': 349.23, 'B': 493.88, 'A': 440, 'g': 831.61, 'f': 698.46, 'a': 880, 'b': 987.77}

mot['beep_amplitude'] = "3"
for letter in music:
    if letter in freq_map:
        mot['beep_frequency'] = str(freq_map[letter])
        mot['beep'] = str(duration)
        time.sleep(duration*1.1)

