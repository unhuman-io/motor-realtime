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

    virtual void set_timeout_ms(int timeout_ms) override;
    virtual int get_timeout_ms() const override { return timeout_ms_; }

    static std::vector<std::string> enumerate_can_devices(std::string interface);
    static int open_socket(std::string if_name);

 private:
    std::string intf_;
    static uint32_t timeout_ms_;
};

}; // namespace obot
