#!/usr/bin/env python3

import motor
import motor_program
import time
import math
import json


def calibrate():
    mot = motor_program.motor_manager.motors()[0]
    print(f"Calibrating {mot.name()}")
    
    motor_program.driver_enable()

    # Get index offset at every motor pole
    motor_program.set_phase_lock()
    time.sleep(2)
    num_poles = int(float(mot["num_poles"].get()))
    print(f"Num motor pole pairs: {num_poles}")
    index_offset_average = 0
    for i in range(0, num_poles):   
        index_offset_measured = float(mot['index_offset_measured'].get())
        print(f"Index offset measured: {i}: {index_offset_measured}")
        motor_position = motor_program.read_average(10)[0].motor_position
        new_motor_position = motor_position + 2*math.pi/num_poles
        motor_program.write_command(motor.ModeDesired.Position, position = [new_motor_position])
        time.sleep(.1)
        motor_program.set_phase_lock()
        time.sleep(.5)
        current_motor_position = motor_program.read_average(10)[0].motor_position
        assert(abs(new_motor_position - current_motor_position)*num_poles < math.pi/4)
        index_offset_average += index_offset_measured/num_poles

    print(f"Index offset average: {index_offset_average}")

    # Set electrical offset, measure voltage limited velocity in both directions
    mot["electrical_zero_pos"] = str(index_offset_average)
    phase_lock_current = float(mot["startup_phase_lock_current"].get())
    motor_program.write_command(motor.ModeDesired.Current, current=[phase_lock_current])
    time.sleep(2)
    positive_velocity = motor_program.read_average(100)[0]

    motor_program.write_command(motor.ModeDesired.Current, current=[-phase_lock_current])
    time.sleep(2)
    negative_velocity = motor_program.read_average(100)[0]

    assert(positive_velocity.motor_velocity > 0)
    assert(negative_velocity.motor_velocity < 0)
    assert((positive_velocity.motor_velocity - negative_velocity.motor_velocity)/positive_velocity.motor_velocity < 0.1)

    assert(positive_velocity.joint_velocity > 0)
    assert(negative_velocity.joint_velocity < 0)
    assert((positive_velocity.joint_velocity - negative_velocity.joint_velocity)/positive_velocity.joint_velocity < 0.1)

    print(f"Done: {mot.name()}")

    json_dict = {"serial_number": mot.serial_number(), "index_offset_measured": index_offset_average}
    print(json.dumps(json_dict))
    

def main():
    motors = motor_program.motor_manager.get_connected_motors()
    for mot in motors:
        motor_program.motor_manager.get_motors_by_path([mot.path()])
        calibrate()


if __name__ == "__main__":
    main()