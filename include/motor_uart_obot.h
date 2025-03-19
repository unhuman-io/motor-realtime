#pragma once

#include "motor.h"
#include "motor_uart.h"
#include "protocol_parser.h"
#include <poll.h>
#include <atomic>
#include <thread>
#include <condition_variable>

namespace obot {

class MotorUARTObot : public Motor {
 public:
    MotorUARTObot(std::string dev_path, uint32_t baud_rate = 4000000);
    virtual ~MotorUARTObot() override;
    
    virtual void set_timeout_ms(int timeout_ms) override;
    void set_baud_rate(uint32_t baud_rate = 4000000);

    virtual ssize_t read() override;
    virtual ssize_t write() override;
    void rx_data();

 private:
    uint32_t timeout_ms_ = 10;
    const static uint32_t RX_BUFFER_SIZE = 2048;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    figure::ProtocolParser parser_{rx_buffer_, RX_BUFFER_SIZE};
    uint8_t rx_lin_buffer_[RX_BUFFER_SIZE];
    std::atomic<uint32_t> current_read_idx_{0};
    std::thread rx_thread_;
    std::atomic<bool> terminate_{false};
    std::condition_variable rx_data_cv_;
    std::mutex rx_data_cv_m_; // protects rx_data_cv_, rx_buf_ and rx_len_
    uint8_t rx_buf_[1024];
    size_t rx_len_ = 0;
};

}; // namespace obot