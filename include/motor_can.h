#pragma once

#include "motor.h"
#include <string>

namespace obot {

class MotorCAN : public Motor {
 public:
    MotorCAN(std::string address);
    virtual ~MotorCAN() {}
    void open();
    virtual ssize_t read() override;
    virtual ssize_t write() override;

 private:
    std::string intf_;
};

}; // namespace obot