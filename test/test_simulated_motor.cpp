#include "motor_manager.h"
#include <cassert>
#include <iostream>
#include <thread>
#include <chrono>

using namespace obot;

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
    std::cout << m.read() << std::endl;
    assert(m.read()[0].torque == 1);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    m.set_command_mode(VELOCITY);
    m.write_saved_commands();
    auto status = m.read();
    std::cout << status << std::endl;
    float motor_position = status[0].motor_position;
    float joint_position = status[0].joint_position;
    assert(std::abs(joint_position*100 - motor_position) < .001);
    m.set_command_velocity({1});
    m.write_saved_commands();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    assert(std::abs(m.read()[0].motor_position - 1 - motor_position) < .1);
    assert(std::abs(m.read()[0].joint_position - 0.01 - joint_position) < .001);
    std::cout << m.read() << std::endl;
    assert(m.read()[0].host_timestamp_received == 6);
    auto motor = m.motors()[0];
    motor->set_timeout_ms(10);
    motor->get_timeout_ms();
    return 0;
}