#pragma once
#include "realtime_thread.h"
#include "motor.h"
#include "motor_manager.h"
#include "cstack.h"
#define MAX_MOTORS 10

class MotorManager;


struct Data {
    Status statuses[MAX_MOTORS];
    Command commands[MAX_MOTORS];
    std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end, aread_time, read_time, control_time, write_time;
    int size() const { return MAX_MOTORS; }
};

class MotorThread : public RealtimeThread {
 public:
    MotorThread(uint32_t frequency_hz = 1000);
    virtual ~MotorThread() {
        finish();
    }
    const CStack<Data> &cstack() const { return cstack_; }
    void init();
    MotorManager& motor_manager() { return motor_manager_; }
 protected:
    virtual void post_init() {}
    virtual void pre_update() {}
    virtual void controller_update() {}
    virtual void post_update() {}
    virtual void finish() {}
    virtual void update() final;
    Data data_;
    MotorManager motor_manager_;
    CStack<Data> cstack_;
    uint32_t poll_timeout_ns_ = 500*1000;
};
