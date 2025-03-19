#include "motor_thread.h"

namespace obot {

MotorThread::MotorThread(uint32_t frequency_hz, bool user_space_driver)
    : RealtimeThread(frequency_hz), motor_manager_(user_space_driver), user_space_driver_(user_space_driver) {
}

void MotorThread::init() {
    std::cout << "Connecting to motors:" << std::endl;
    for (auto m : motor_manager_.motors()) {
        std:: cout << m->name() << std::endl;
    }
    post_init();
    for (int i=0; i<motor_manager_.size(); i++) {
        cpu_frequency_hz_[i] = motor_manager_.motors()[i]->get_cpu_frequency();
    }
    std::memcpy(data_.statuses, motor_manager_.read().data(), sizeof(MotorStatus)*motor_manager_.size());
    cstack_.push(data_);
}

void MotorThread::update() {
    data_.last_time_start = data_.time_start;
    data_.time_start = std::chrono::steady_clock::now();
    // start a read on all motors
    if (use_poll_) {
        motor_manager_.multipoll(1);
    }
    data_.aread_time = std::chrono::steady_clock::now();

    // there is some time before data will return on USB, can do pre update work
    pre_update();

    // poll with timeout
    if (use_poll_) {
        int retval = motor_manager_.multipoll(poll_timeout_ns_);
        if (retval != motor_manager_.size()) {
            if (!user_space_driver_) {
                // user space driver currently doesn't deal with poll
                throw std::runtime_error("MotorThread poll error " + std::to_string(retval) + " " + strerror(-retval));
            }
        }
    }

    // blocking io to get the data already set up and wait if not ready yet
    std::memcpy(data_.statuses, motor_manager_.read().data(), sizeof(MotorStatus)*motor_manager_.size());
    data_.read_time = std::chrono::steady_clock::now();

    Data last_data = cstack_.top();
    for (int i=0; i < motor_manager_.size(); i++) {
        data_.controller_dt_s[i] = (data_.statuses[i].mcu_timestamp - last_data.statuses[i].mcu_timestamp)/cpu_frequency_hz_[i];
    }

    controller_update();
    data_.control_time = std::chrono::steady_clock::now();

    motor_manager_.write_saved_commands();
    std::memcpy(data_.commands, motor_manager_.commands().data(), sizeof(MotorCommand)*motor_manager_.size());
    data_.write_time = std::chrono::steady_clock::now();

    RealtimeThread::update();
    
    post_update();
    cstack_.push(data_);
    data_.last_time_end = std::chrono::steady_clock::now();
}

}  // namespace obot
