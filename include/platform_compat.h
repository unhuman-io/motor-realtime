#pragma once

// Small compatibility shims for building on macOS, which lacks a few
// Linux/POSIX-RT libc functions used by this project. Entirely empty on Linux.
#ifdef __APPLE__

#include <poll.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

// macOS has no ppoll(). Emulate with poll(). The sigmask argument is not
// supported (every call site in this project passes nullptr). Defined at
// global scope because call sites use ::ppoll(...). Note: a function, not a
// macro, so it does not collide with the `bool ppoll` option flag in
// motor_util.cpp.
static inline int ppoll(struct pollfd *fds, nfds_t nfds,
                        const struct timespec *timeout_ts,
                        const sigset_t * /*sigmask*/) {
    int timeout_ms = -1;
    if (timeout_ts != nullptr) {
        timeout_ms = static_cast<int>(timeout_ts->tv_sec * 1000 +
                                      (timeout_ts->tv_nsec + 999999) / 1000000);
    }
    return ::poll(fds, nfds, timeout_ms);
}

#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

// macOS has no clock_nanosleep(). Emulate with nanosleep(). Supports
// TIMER_ABSTIME against CLOCK_MONOTONIC, which is all this project uses.
// Mirrors the real clock_nanosleep() contract: returns 0 on success or a
// positive errno on failure (it never returns -1 / sets errno).
static inline int clock_nanosleep(clockid_t clock_id, int flags,
                                  const struct timespec *req,
                                  struct timespec *rem) {
    struct timespec relative;
    if (flags & TIMER_ABSTIME) {
        struct timespec now;
        clock_gettime(clock_id, &now);
        relative.tv_sec = req->tv_sec - now.tv_sec;
        relative.tv_nsec = req->tv_nsec - now.tv_nsec;
        if (relative.tv_nsec < 0) {
            relative.tv_nsec += 1000000000L;
            relative.tv_sec -= 1;
        }
        if (relative.tv_sec < 0) {
            return 0;  // target time already in the past
        }
    } else {
        relative = *req;
    }
    int r = ::nanosleep(&relative, rem);
    return r == 0 ? 0 : errno;
}

#endif  // __APPLE__
