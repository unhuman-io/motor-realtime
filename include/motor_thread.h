#pragma once
#include "realtime_thread.h"
#include <vector>
#include "motor.h"
#include "motor_manager.h"

class MotorManager;

struct Data {
    std::vector<Status> statuses;
    std::vector<Command> commands;
    std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end, aread_time, read_time, control_time, write_time;
};

// A circular stack. If data is written by one thread and read by one other thread, data is read from the top without worrying about thread safety.
template <class T>
class CStack {
 public:
    void push(T &t) {
		int future_pos = pos_ + 1;
		if (future_pos >= 100) {
			future_pos = 0;
		}
		data_[future_pos] = t;
		pos_ = future_pos;
	}
	T top() const {
		return data_[pos_];
	}
 private:
	T data_[100];
	int pos_ = 0;
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