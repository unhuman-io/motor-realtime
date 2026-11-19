#pragma once

#include "exception.h"
#include "motor_messages.h"
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <time.h>
#include <vector>

namespace obot {

class MotorDescription {
 public:
    std::string name() const { return name_; }
    std::string serial_number() const { return serial_number_; }
    std::string base_path() const {return base_path_; }
    std::string dev_path() const { return dev_path_; }
    uint32_t devnum() const { return devnum_; }
    std::string version() const { return version_; }
    std::string board_name() const { return board_name_; }
    std::string board_rev() const { return board_rev_; }
    std::string board_num() const { return board_num_; }
    std::string messages_version() const { return messages_version_; }
    std::string config() const { return config_; }
    // The description out of a node's enum reply, as the text api gives it. A
    // node leaves what it doesn't know zero, which is an empty string here.
    void parse_enum(const EnumResponse &e) {
      auto text = [](const char *s, size_t size) {
        return std::string(s, strnlen(s, size));
      };
      char c[40];
      messages_version_ = std::to_string(e.messages_version[0]) + "." +
                          std::to_string(e.messages_version[1]);
      std::snprintf(c, sizeof(c), "%02x%02x%02x%02x%02x%02x",
                    e.serial_number[0], e.serial_number[1], e.serial_number[2],
                    e.serial_number[3], e.serial_number[4], e.serial_number[5]);
      serial_number_ = c;
      std::snprintf(c, sizeof(c), "v%u.%u-%u-g%07x%s", e.version[0],
                    e.version[1], e.version_commits,
                    static_cast<unsigned>(e.version_hash & ~ENUM_VERSION_DIRTY),
                    e.version_hash & ENUM_VERSION_DIRTY ? "-dirty" : "");
      version_ = c;
      name_ = text(e.name, sizeof(e.name));
      board_name_ = text(e.board_name, sizeof(e.board_name));
      board_rev_ = board_name_.empty() ? "" : std::to_string(e.board_rev);
      board_num_ = board_name_.empty() ? "" : std::to_string(e.board_num);
      config_ = "";
    }
    virtual std::string short_version() const {
        std::string s = version();
        if (auto pos = s.find("~"); pos != std::string::npos) {
            return s.substr(0,5) + s.substr(pos,6);
        }
        auto pos = std::min(s.find(" "), s.find("-g"));
        return s.substr(0,pos);
    }
    virtual void set_timeout_ms(int timeout_ms) {};
    virtual int get_timeout_ms() const { return 0; };
 protected:
    std::string name_, serial_number_, base_path_, dev_path_, version_;
    std::string board_name_, board_rev_, board_num_, messages_version_, config_;
    uint32_t devnum_;
};

// up to 2 seconds
class Timer {
 public:
    Timer(uint32_t timeout_ns) : timeout_ns_(timeout_ns) {
        time_start_ = get_time();
    }

    uint32_t get_time_remaining_ns() const {
        struct timespec time_now = get_time();
        uint32_t time_elapsed = (time_now.tv_sec - time_start_.tv_sec) * 1e9 +
             (time_now.tv_nsec - time_start_.tv_nsec);
        int32_t time_remaining = timeout_ns_ - time_elapsed;
        if (time_remaining < 0) {
            time_remaining = 0;
        }
        return time_remaining;
    }
 private:
    struct timespec get_time() const {
        struct timespec t;
        int retval = clock_gettime(CLOCK_MONOTONIC, &t);
        if (retval < 0) {
            throw RuntimeException("clock get time error");
        }
        return t;
    }
    struct timespec time_start_;
    uint32_t timeout_ns_;
};


std::vector<std::string> udev_list_dfu();
class DFUDevice : public MotorDescription {
 public:
    DFUDevice(std::string dev_path);
};

std::string short_status(std::vector<Status> statuses);
std::string get_config_dir();
void interruptible_sleep_until(std::chrono::steady_clock::time_point next_time);

}  // namespace obot
