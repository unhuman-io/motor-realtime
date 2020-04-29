#include "motor_thread.h"

MotorThread::MotorThread(uint32_t frequency_hz)
    : RealtimeThread(frequency_hz) {
}

void MotorThread::init() {
    motor_manager_.open();
    std::cout << "Connecting to motors:" << std::endl;
    for (auto m : motor_manager_.motors()) {
        std:: cout << m->name() << std::endl;
    }
    data_.commands.resize(motor_manager_.motors().size());
    data_.statuses.resize(motor_manager_.motors().size());
}

void MotorThread::update() {
    data_.last_time_start = data_.time_start;
    data_.time_start = std::chrono::steady_clock::now();
    // start a read on all motors
    motor_manager_.aread();
    data_.aread_time = std::chrono::steady_clock::now();

    // there is some time before data will return on USB, can do pre update work
    pre_update();
    // blocking io to get the data already set up and wait if not ready yet
    data_.statuses = motor_manager_.read();
    data_.read_time = std::chrono::steady_clock::now();

    controller_update();
    data_.control_time = std::chrono::steady_clock::now();

    motor_manager_.write_saved_commands();
    data_.commands = motor_manager_.commands();
    data_.write_time = std::chrono::steady_clock::now();

    post_update();
    cstack_.push(data_);
    RealtimeThread::update();
    data_.last_time_end = std::chrono::steady_clock::now();
}