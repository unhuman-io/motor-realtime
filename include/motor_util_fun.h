#pragma once

#include <time.h>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "motor_messages.h"

namespace obot {

class MotorDescription {
 public:
    std::string name() const { return name_; }
    std::string serial_number() const { return serial_number_; }
    std::string base_path() const {return base_path_; }
    std::string dev_path() const { return dev_path_; }
    uint8_t devnum() const { return devnum_; }
    std::string version() const { return version_; }
    virtual std::string short_version() const { return version_; }
    virtual void set_timeout_ms(int timeout_ms) {};
    virtual int get_timeout_ms() const { return 0; };
 protected:
    std::string name_, serial_number_, base_path_, dev_path_, version_;
    uint8_t devnum_;
};

// up to 2 seconds
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

std::vector<std::string> udev_list_dfu();
class DFUDevice : public MotorDescription {
 public:
    DFUDevice(std::string dev_path);
};

std::string short_status(std::vector<Status> statuses);

}  // namespace obot
