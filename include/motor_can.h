#pragma once

#include "motor.h"
#include <string>
#include <vector>

namespace obot {

class MotorCAN : public Motor {
 public:
    MotorCAN(std::string address);
    virtual ~MotorCAN() {}
    void open();
    virtual ssize_t read() override;
    virtual ssize_t write() override;
    virtual ssize_t aread() override {
        send_read_request_ = false;
        return 0;
    }

    // Nonblocking, as for a motor on USB: read() does not wait for a status. It takes what
    // has arrived and returns -1 with errno EAGAIN if that is nothing. Unless the node
    // answers every command with a status (set_cmd_status_req()), read() still asks it
    // for one; the answer is there for the next read().
    virtual int set_nonblock() override {
        nonblock_ = true;
        return 0;
    }
    virtual int clear_nonblock() override {
        nonblock_ = false;
        return 0;
    }

    virtual void set_timeout_ms(int timeout_ms) override;
    virtual int get_timeout_ms() const override { return timeout_ms_; }

    static std::vector<std::string> enumerate_can_devices(std::string interface);
    static int open_socket(std::string if_name);

    // Timeout used by enumerate_can_devices() and by every CANFile opened afterwards.
    // Settable before any MotorCAN exists, unlike the set_timeout_ms() override.
    static void set_default_timeout_ms(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }
    static uint32_t get_default_timeout_ms() { return timeout_ms_; }

 private:
    std::string intf_;
    static uint32_t timeout_ms_;
    bool send_read_request_ = true;
};

}; // namespace obot
