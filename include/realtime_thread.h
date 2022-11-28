#pragma once
#include <cstdint>
#include <future>
#include <chrono>
#include <functional>

namespace std {
    class thread;
}

namespace obot {

class RealtimeThread {
 public:
	RealtimeThread(uint32_t frequency_hz, std::function<void ()> update_fun = [](){}, bool debug = false) 
        : update_fun_(update_fun), debug_(debug) {
		period_ns_ = static_cast<uint32_t>(1.0e9/frequency_hz);
        max_jitter_ns_ = static_cast<uint32_t>(.1*period_ns_);
	}
    virtual ~RealtimeThread() {}
    void run();
	void done();
 protected:
    virtual void update() { update_fun_(); }
    virtual void jitter_error(int32_t time_jitter_ns);
    std::chrono::steady_clock::time_point start_time_;
 private:
    void run_deadline();
    std::thread *thread_;
	uint32_t period_ns_;
    int32_t max_jitter_ns_;
    bool done_ = false;
    std::function<void ()> update_fun_;
    std::promise<void> exit_;
    uint32_t jitter_error_ = 0;
    bool debug_;
};

}  // namespace obot
