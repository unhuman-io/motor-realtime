#pragma once
#include <cstdint>

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
 private:
    void run_deadline();
    std::thread *thread_;
	uint32_t period_ns_;
    bool done_ = false;
};