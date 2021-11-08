#pragma once

#include <time.h>
#include <cstdint>
#include <stdexcept>

class Timer {
 public:
    Timer(uint32_t timeout_ns) : timeout_ns_(timeout_ns) {
        time_start_ = get_time_ns();
    }

    uint32_t get_time_remaining_ns() const {
        uint32_t time_elapsed = get_time_ns() - time_start_;
        int32_t time_remaining = timeout_ns_ - time_elapsed;
        if (time_remaining < 0) {
            time_remaining = 0;
        }
        return time_remaining;
    }
 private:
    uint32_t get_time_ns() const {
        struct timespec t;
        int retval = clock_gettime(CLOCK_MONOTONIC, &t);
        if (retval < 0) {
            throw std::runtime_error("clock get time error");
        }
        return t.tv_nsec;
    }
    uint32_t time_start_;
    uint32_t timeout_ns_;
};
