#include "motor_manager.h"
#include <cassert>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    MotorManager m;
    m.get_motors_by_name({"sim"}, true, true);
    assert(typeid(*(m.motors()[0].get())) == typeid(SimulatedMotor));
    (dynamic_cast<SimulatedMotor *>(m.motors()[0].get()))->set_gear_ratio(100);
    m.set_auto_count();
    assert(m.read()[0].iq == 0);
    std::cout << m.read() << std::endl;
    m.set_command_current({1});
    m.write_saved_commands();
    assert(m.read()[0].iq == 0);
    m.set_command_mode(CURRENT);
    m.write_saved_commands();
    assert(m.read()[0].iq == 1);
    m.set_command_mode(POSITION);
    m.set_command_position({1});
    m.write_saved_commands();
    assert(m.read()[0].motor_position == 1);
    std::cout << m.read() << std::endl;
    assert(m.read()[0].joint_position == 0.01f);
    m.set_command_mode(TORQUE);
    m.set_command_torque({1});
    m.write_saved_commands();
    assert(m.read()[0].torque == 1);
    m.set_command_mode(VELOCITY);
    m.write_saved_commands();
    assert(m.read()[0].motor_position == 1);
    assert(m.read()[0].joint_position == 0.01f);
    m.set_command_velocity({1});
    m.write_saved_commands();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    assert(abs(m.read()[0].motor_position - 2) < .1);
    assert(abs(m.read()[0].joint_position - 0.02) < .001);
    std::cout << m.read() << std::endl;
    assert(m.read()[0].host_timestamp_received == 6);
    return 0;
}