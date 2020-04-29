#pragma once
#include <cstdint>
#include <future>
#include <chrono>

namespace std {
    class thread;
}

class RealtimeThread {
 public:
	RealtimeThread(uint32_t frequency_hz) {
		period_ns_ = 1.0e9/frequency_hz;
	}
    void run();
	void done();
 protected:
    virtual void update() {}
    std::chrono::steady_clock::time_point start_time_;
 private:
    void run_deadline();
    std::thread *thread_;
	uint32_t period_ns_;
    bool done_ = false;
    std::promise<void> exit_;
};