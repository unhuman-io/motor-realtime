#pragma once
#include <cstdint>
#include <future>
#include <chrono>
#include <functional>
#include <atomic>
#include <string>
#include <vector>

namespace std {
class thread;
}

namespace obot {

class RealtimeThread {
 public:
    // How the thread gets its time.
    // DEADLINE is the default and what this class has always done: SCHED_DEADLINE, or
    //   plain sleeping until each period where that is not permitted.
    // FIFO is SCHED_FIFO at a fixed priority, sleeping until each period. It is the
    //   choice for a thread pinned to a CPU that is kept free for it (isolcpus), which
    //   SCHED_DEADLINE does not allow: the kernel refuses it for a thread with a
    //   restricted CPU affinity.
    // NORMAL sleeps until each period without any realtime scheduling.
    // With FIFO and NORMAL a cycle that ran past its period is not followed by a burst
    // of cycles to catch up: the periods that were missed are skipped and counted.
    enum Scheduler { DEADLINE,
                     FIFO,
                     NORMAL };

    RealtimeThread(uint32_t frequency_hz, std::function<void()> update_fun = []() {}, bool debug = false) :
        update_fun_(update_fun),
        debug_(debug) {
        set_frequency(frequency_hz);
    }
    virtual ~RealtimeThread() {}
    void set_frequency(uint32_t frequency_hz) {
        period_ns_ = static_cast<uint32_t>(1.0e9 / frequency_hz);
        max_jitter_ns_ = static_cast<uint32_t>(.1 * period_ns_);
    }
    uint32_t get_frequency() const {
        return 1.0e9 / period_ns_;
    }
    // For periods that are not a whole number of Hz.
    void set_period_ns(uint32_t period_ns) {
        period_ns_ = period_ns;
        max_jitter_ns_ = static_cast<uint32_t>(.1 * period_ns_);
    }
    uint32_t get_period_ns() const { return period_ns_; }
    // These are set before run().
    void set_scheduler(Scheduler scheduler, int fifo_priority = 80) {
        scheduler_ = scheduler;
        fifo_priority_ = fifo_priority;
    }
    // FIFO and NORMAL: run on this CPU only; -1 (the default) for any.
    void set_cpu(int cpu) { cpu_ = cpu; }
    void set_lock_memory(bool lock_memory) { lock_memory_ = lock_memory; }

    // Starts the thread and returns when it has set itself up.
    void run();
    void done();

    // What the thread could not set up when it started (no permission for the scheduler,
    // memory not locked, ...), in words. Empty if all went well, and before run().
    const std::vector<std::string>& setup_errors() const { return setup_errors_; }
    // Cycles whose period was off by more than 10 %.
    uint32_t jitter_errors() const { return jitter_error_.load(); }
    // FIFO and NORMAL: periods skipped because a cycle ran past its period.
    uint32_t overruns() const { return overruns_.load(); }

 protected:
    virtual void update() { update_fun_(); }
    virtual void jitter_error(int32_t time_jitter_ns);
    std::chrono::steady_clock::time_point start_time_;
    // For update(), with FIFO and NORMAL: how long after its scheduled time this cycle
    // started, and whether the cycle before it ran past its period.
    int64_t wake_latency_ns_ = 0;
    bool overrun_ = false;

 private:
    void thread_main();
    void setup();
    void run_deadline(bool deadline_permissions);
    void run_periodic();
    std::thread* thread_ = nullptr;
    uint32_t period_ns_;
    int32_t max_jitter_ns_;
    std::atomic<bool> done_{false};
    std::function<void()> update_fun_;
    std::promise<void> exit_;
    std::promise<void> started_;
    std::atomic<uint32_t> jitter_error_{0};
    std::atomic<uint32_t> overruns_{0};
    std::vector<std::string> setup_errors_;
    Scheduler scheduler_ = DEADLINE;
    int fifo_priority_ = 80;
    int cpu_ = -1;
    bool lock_memory_ = true;
    bool deadline_permissions_ = false;
    bool debug_;
};

}  // namespace obot
