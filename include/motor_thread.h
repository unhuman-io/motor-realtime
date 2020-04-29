#pragma once
#include "realtime_thread.h"
#include <vector>
#include "motor.h"

class MotorManager;

struct Data {
    std::vector<Status> statuses;
    std::vector<Command> commands;
    std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end, aread_time, read_time, control_time, write_time;
};

class MotorThread : public RealtimeThread {
 public:
    MotorThread(MotorManager &motor_manager, uint32_t frequency_hz = 1000);
 protected:
    virtual void pre_update() {}
    virtual void controller_update() {}
    virtual void post_update() {}
    virtual void update();
    Data data_;
    MotorManager &motor_manager_;
};