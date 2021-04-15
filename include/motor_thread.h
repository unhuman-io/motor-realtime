#pragma once
#include "realtime_thread.h"
#include <vector>
#include "motor.h"
#include "motor_manager.h"
#include <atomic>
#include "cstack.h"

class MotorManager;

struct Data {
    std::vector<Status> statuses;
    std::vector<Command> commands;
    std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end, aread_time, read_time, control_time, write_time;
};

class MotorThread : public RealtimeThread {
 public:
    MotorThread(uint32_t frequency_hz = 1000);
    const CStack<Data> &cstack() const { return cstack_; }
    void init();
    MotorManager& motor_manager() { return motor_manager_; }
 protected:
    virtual void post_init() {}
    virtual void pre_update() {}
    virtual void controller_update() {}
    virtual void post_update() {}
    virtual void update();
    Data data_;
    MotorManager motor_manager_;
    CStack<Data> cstack_;
};