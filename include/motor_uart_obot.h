#pragma once

#include "motor.h"
#include "motor_uart.h"
#include <protocol_parser.h>
#include <poll.h>

namespace obot {

#define RX_BUFFER_SIZE 2048

class MotorUARTObot : public Motor {
 public:
    MotorUARTObot(std::string dev_path, uint32_t baud_rate = 4000000);
    //virtual ~MotorUART() override;
    
    virtual void set_timeout_ms(int timeout_ms) override;
    void set_baud_rate(uint32_t baud_rate = 4000000);

    virtual ssize_t read() override;
    virtual ssize_t write() override;

 private:
    Mailbox realtime_mailbox_;
    uint32_t timeout_ms_ = 10;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    figure::ProtocolParser parser_{rx_buffer_, RX_BUFFER_SIZE};
};

}; // namespace obot