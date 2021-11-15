#include "motor_thread.h"

MotorThread::MotorThread(uint32_t frequency_hz)
    : RealtimeThread(frequency_hz) {
}

void MotorThread::init() {
    std::cout << "Connecting to motors:" << std::endl;
    for (auto m : motor_manager_.motors()) {
        std:: cout << m->name() << std::endl;
    }
    post_init();
}

void MotorThread::update() {
    data_.last_time_start = data_.time_start;
    data_.time_start = std::chrono::steady_clock::now();
    // start a read on all motors
    motor_manager_.multipoll(0);
    data_.aread_time = std::chrono::steady_clock::now();

    // there is some time before data will return on USB, can do pre update work
    pre_update();

    // poll with timeout
    int retval = motor_manager_.multipoll(poll_timeout_ns_);
    if (retval != motor_manager_.motors().size()) {
        throw std::runtime_error("MotorThread poll error " + std::to_string(retval) + " " + strerror(-retval));
    }

    // blocking io to get the data already set up and wait if not ready yet
    std::memcpy(data_.statuses, motor_manager_.read().data(), sizeof(data_.statuses));
    data_.read_time = std::chrono::steady_clock::now();

    controller_update();
    data_.control_time = std::chrono::steady_clock::now();

    motor_manager_.write_saved_commands();
    std::memcpy(data_.commands, motor_manager_.commands().data(), sizeof(data_.commands));
    data_.write_time = std::chrono::steady_clock::now();

    RealtimeThread::update();
    
    post_update();
    cstack_.push(data_);
    data_.last_time_end = std::chrono::steady_clock::now();
}