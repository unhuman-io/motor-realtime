#include "realtime_thread.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sched.h>
#include <errno.h>

#include <chrono>
#include <thread>
#include <iostream>

#define gettid()       syscall(__NR_gettid)

#define SCHED_DEADLINE 6

/* XXX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr 314
#define __NR_sched_getattr 315
#endif

#ifdef __i386__
#define __NR_sched_setattr 351
#define __NR_sched_getattr 352
#endif

#ifdef __arm__
#ifndef __NR_sched_setattr
#define __NR_sched_setattr 380
#endif
#ifndef __NR_sched_getattr
#define __NR_sched_getattr 381
#endif
#endif

namespace obot {

struct sched_attr {
    __u32 size;

    __u32 sched_policy;
    __u64 sched_flags;

    /* SCHED_NORMAL, SCHED_BATCH */
    __s32 sched_nice;

    /* SCHED_FIFO, SCHED_RR */
    __u32 sched_priority;

    /* SCHED_DEADLINE (nsec) */
    __u64 sched_runtime;
    __u64 sched_deadline;
    __u64 sched_period;
};

int sched_setattr(pid_t pid,
                  const struct sched_attr* attr,
                  unsigned int flags) {
    return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid,
                  struct sched_attr* attr,
                  unsigned int size,
                  unsigned int flags) {
    return syscall(__NR_sched_getattr, pid, attr, size, flags);
}


void RealtimeThread::run() {
    done_ = false;
    // before the thread exists: done() may be called before the thread has got anywhere
    exit_ = std::promise<void>();
    started_ = std::promise<void>();
    setup_errors_.clear();
    thread_ = new std::thread([this] { thread_main(); });
    started_.get_future().wait();
}

void RealtimeThread::done() {
    if (thread_ == nullptr) {
        // never started, or stopped already
        return;
    }
    done_.store(true);
    if (exit_.get_future().wait_for(std::chrono::nanoseconds(period_ns_) * 20) == std::future_status::timeout) {
        std::cerr << "Difficulty stopping realtime thread" << std::endl;
        // It may be stuck in update(). A thread that is neither joined nor detached ends
        // the program when it is deleted.
        thread_->detach();
    } else {
        thread_->join();
        //printf("Realtime thread joined\n");
    }
    delete thread_;
    thread_ = nullptr;
}

void RealtimeThread::jitter_error(int32_t time_jitter_ns) {
    jitter_error_++;
}

void RealtimeThread::setup() {
    auto failed = [this](const std::string& what) {
        setup_errors_.push_back(what + ": " + strerror(errno));
        if (debug_) {
            std::cerr << "RealtimeThread: " << setup_errors_.back() << std::endl;
        }
    };

    deadline_permissions_ = false;
    if (scheduler_ == DEADLINE) {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        attr.sched_flags = 0;
        attr.sched_nice = 0;
        attr.sched_priority = 0;

        attr.sched_policy = SCHED_DEADLINE;
        attr.sched_runtime = period_ns_ * 9.0 / 10;
        attr.sched_deadline = period_ns_ * 9.0 / 10;
        attr.sched_period = period_ns_;

        unsigned int flags = 0;
        if (sched_setattr(0, &attr, flags) < 0) {
            failed("SCHED_DEADLINE not set, sleeping until each period instead");
        } else {
            deadline_permissions_ = true;
            if (debug_) {
                printf("Running deadline scheduler\n");
            }
        }
    } else {
        if (cpu_ >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(cpu_, &set);
            if (sched_setaffinity(0, sizeof(set), &set) < 0) {
                failed("not pinned to cpu " + std::to_string(cpu_));
            }
        }
        if (scheduler_ == FIFO) {
            struct sched_param param = {};
            param.sched_priority = fifo_priority_;
            if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
                failed("SCHED_FIFO priority " + std::to_string(fifo_priority_) + " not set");
            }
        }
    }

    if (lock_memory_ && mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
        failed("memory not locked");
    }
}

void RealtimeThread::thread_main() {
    //printf("realtime thread started period_ns = %d, [%ld]\n", period_ns_, gettid());
    setup();
    started_.set_value();
    if (scheduler_ == DEADLINE) {
        run_deadline(deadline_permissions_);
    } else {
        run_periodic();
    }
    exit_.set_value();
    //printf("realtime thread finish [%ld]\n", gettid());
}

void RealtimeThread::run_deadline(bool deadline_permissions) {
    auto next_time = std::chrono::steady_clock::now();
    start_time_ = next_time;
    auto last_loop_start_time = start_time_ - std::chrono::nanoseconds(period_ns_);
    while (!done_.load()) {
        auto loop_start_time = std::chrono::steady_clock::now();
        auto period = loop_start_time - last_loop_start_time;
        int32_t time_jitter = std::chrono::duration_cast<std::chrono::nanoseconds>(period).count() - period_ns_;


        if (abs(time_jitter) > max_jitter_ns_) {
            jitter_error(time_jitter);
        }

        update();

        last_loop_start_time = loop_start_time;
        next_time += std::chrono::nanoseconds(period_ns_);
        if (!deadline_permissions) {
            std::this_thread::sleep_until(next_time);
        } else {
            sched_yield();
        }
    }
}

// Each cycle starts at a time fixed in advance, so being late once does not shift the
// cycles after it.
void RealtimeThread::run_periodic() {
    const std::chrono::nanoseconds period(period_ns_);
    start_time_ = std::chrono::steady_clock::now();
    auto next_time = start_time_ + period;
    auto last_loop_start_time = start_time_;
    while (!done_.load()) {
        // steady_clock is CLOCK_MONOTONIC; sleeping to an absolute time on it directly
        // does not depend on how the standard library implements sleep_until
        auto wake_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(next_time.time_since_epoch()).count();
        struct timespec wake = {};
        wake.tv_sec = wake_ns / 1000000000;
        wake.tv_nsec = wake_ns % 1000000000;
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR) {
        }
        auto loop_start_time = std::chrono::steady_clock::now();
        wake_latency_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - next_time).count();
        int32_t time_jitter = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - last_loop_start_time).count() - period_ns_;
        if (abs(time_jitter) > max_jitter_ns_) {
            jitter_error(time_jitter);
        }

        update();

        last_loop_start_time = loop_start_time;
        next_time += period;
        auto now = std::chrono::steady_clock::now();
        overrun_ = now > next_time;
        if (overrun_) {
            // skip the periods that were missed instead of running a burst to catch up
            auto missed = (now - next_time) / period + 1;
            next_time += missed * period;
            overruns_ += missed;
        }
    }
}

}  // namespace obot
