#pragma once

#include "motor.h"

namespace obot {

class MotorUART : public Motor {
 public:
    MotorUART(std::string dev_path);
    //virtual ~MotorUART() override;
    
    virtual void set_timeout_ms(int timeout_ms) override;
    void set_baud_rate(uint32_t baud_rate = 4000000);

    virtual ssize_t read() override;
    virtual ssize_t write() override;

 private:
    uint32_t read_error_ = 0;
};

}; // namespace obot