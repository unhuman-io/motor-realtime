import motor
import sys
import signal
import time

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

def read_average(num_read = 1):
    status = motor_manager.read_average(num_read)
    return status