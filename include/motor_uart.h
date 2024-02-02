#pragma once

#include "motor.h"
#include <poll.h>

namespace obot {

class Mailbox : public TextFile {
 public:
    virtual ssize_t read(char * data, unsigned int length) override;
    virtual ssize_t write(const char * data, unsigned int length) override;
    virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) override;
    uint8_t send_mailbox_ = 1; // realtime command
    uint8_t recv_mailbox_ = 2; // realtime status
    uint8_t send_recv_mailbox_ = 3; // both
    int fd_;
 private:
    uint32_t read_error_ = 0;
    uint32_t write_error_ = 0;
};

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
    Mailbox realtime_mailbox_;
    struct pollfd pollfds_[1] = {};
    uint32_t sync_error_ = 0;
};

}; // namespace obot