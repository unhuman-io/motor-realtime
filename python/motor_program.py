import motor
import sys
import signal
import time
import numpy as np
from io import StringIO

motor_manager = motor.MotorManager()

def stop():
    print('Stopping')
    motor_manager.set_command_mode(motor.ModeDesired.Open)
    motor_manager.write_saved_commands()

def signal_handler(sig, frame):
    stop()
    sys.exit(0) 

signal.signal(signal.SIGINT, signal_handler)

def connected_names():
    return " ".join([x.name() for x in motor_manager.motors()])

def get_all_api(string):
    return [x[string].get() for x in motor_manager.motors()]

def get_fast_loop_status():
        out = []
        for mot in motor_manager.motors():
            status = mot["fast_loop_status"].get()
            out.append(np.genfromtxt(StringIO(status), delimiter=','))
            # status.timestamp,
            #         status.foc_command.measured.motor_encoder,
            #         status.foc_command.desired.i_q,
            #         status.foc_status.measured.i_q,
            #         status.foc_command.measured.i_a,
            #         status.foc_command.measured.i_b,
            #         status.foc_command.measured.i_c,
            #         status.foc_status.command.v_a,
            #         status.foc_status.command.v_b,
            #         status.foc_status.command.v_c,
            #         status.vbus);
        return out

def driver_enable():
    print(f"Enabling: " + connected_names())
    write_command(motor.ModeDesired.DriverEnable)
    time.sleep(0.01)

def set_phase_lock():
    phase_lock_current = [float(x) for x in get_all_api("startup_phase_lock_current")]
    write_command(motor.ModeDesired.PhaseLock, current = phase_lock_current)

def write_command(mode=None, current=None, position=None):
    motor_manager.clear_commands()
    if mode:
        motor_manager.set_command_mode(mode)
    if current:
        motor_manager.set_command_current(current)
    if position:
        motor_manager.set_command_position(position)
    motor_manager.write_saved_commands()

def write_stepper_velocity(current=0, velocity=0):
    motor_manager.set_command_stepper_velocity(current, velocity)
    motor_manager.write_saved_commands()

def read_average(num_read = 1):
    status = motor_manager.read_average(num_read)
    return status