#pragma once

#include "motor.h"
#include <poll.h>

namespace obot {

class MotorUART : public Motor {
 public:
    MotorUART(std::string dev_path, uint32_t baud_rate = 4000000);
    //virtual ~MotorUART() override;
    
    virtual void set_timeout_ms(int timeout_ms) override;
    void set_baud_rate(uint32_t baud_rate = 4000000);

    virtual ssize_t read() override;
    virtual ssize_t write() override;
    int sync();

 private:
    uint32_t read_error_ = 0;
    uint32_t write_error_ = 0;
    uint32_t sync_error_ = 0;
    struct pollfd pollfds_[1] = {};
};

}; // namespace obot