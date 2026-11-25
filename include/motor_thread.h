#pragma once
#include "realtime_thread.h"
#include "motor.h"
#include "motor_manager.h"
#include "cstack.h"
// The most motors a MotorThread handles. A program that needs more defines it for the
// whole build, library included.
#ifndef MAX_MOTORS
#define MAX_MOTORS 10
#endif

namespace obot {

struct Data {
    Status statuses[MAX_MOTORS];
    Command commands[MAX_MOTORS];
    double controller_dt_s[MAX_MOTORS];
    std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end, aread_time, read_time, control_time, write_time;
    // Fault tolerant mode, see MotorThread::set_fault_tolerant():
    bool status_fresh[MAX_MOTORS];   // the status is new in this cycle
    bool write_failed[MAX_MOTORS];   // the command of the cycle before could not be written
    int size() const { return MAX_MOTORS; }
};

class MotorThread : public RealtimeThread {
 public:
    MotorThread(uint32_t frequency_hz = 1000, bool user_space_driver = false);
    virtual ~MotorThread() {
        finish();
    }
    const CStack<Data>& cstack() const { return cstack_; }
    void init();
    void set_no_poll() { use_poll_ = false; }
    void set_poll_timeout(uint32_t ns) { poll_timeout_ns_ = ns; }
    // For a program that has to keep running when a motor is late or gone, like the
    // control loop of a robot: set before init(). A motor that has not answered by the
    // poll timeout, or that cannot be read or written, then no longer ends the thread
    // with an exception. The cycle goes on with the others, and controller_update()
    // finds in data_.status_fresh which statuses are new and in data_.write_failed whose
    // command did not go out. For that the motors are made nonblocking and are asked
    // to answer every command with a status (Motor::set_cmd_status_req), which is how a
    // node on CAN gets to send statuses without being asked for each.
    void set_fault_tolerant(bool fault_tolerant = true) { fault_tolerant_ = fault_tolerant; }
    MotorManager& motor_manager() { return motor_manager_; }
    virtual void keyboard_press(char c) {}

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
    uint32_t poll_timeout_ns_ = 500 * 1000;
    bool user_space_driver_;
    bool use_poll_ = true;
    bool fault_tolerant_ = false;
    double cpu_frequency_hz_[MAX_MOTORS];
};

}  // namespace obot
