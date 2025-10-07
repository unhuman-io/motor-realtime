#pragma once

#include "motor.h"

#include <netdb.h>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace obot {

class SocketFile : public TextFile {
 public:
    SocketFile() {}
    virtual ~SocketFile() override {}
    int poll();
    virtual void flush();
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false);
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false);
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;

    void set_api_mode() { api_mode_ = true; }
    int lock_communication();
    int unlock_communication();
    int fd_;
    int timeout_ms_ = 50;
    int fd_communication_lock_;
    std::atomic<int> communication_lock_count_{};
    std::string address_;
    void rx_callback(const uint8_t*, uint16_t);
 protected:
    virtual ssize_t _read(char * /* data */, unsigned int /* length */, bool write_read = false);
 private:
    std::condition_variable rx_data_cv_;
    std::mutex rx_data_cv_m_; // protects rx_data_cv_, rx_buf_, rx_received_ and rx_len_
    uint8_t rx_buf_[1024];
    size_t rx_len_ = 0;
    bool rx_received_ = false;
    std::condition_variable rx_data_request_cv_;
    std::mutex rx_data_request_cv_m_; // protects rx_data_request_
    bool rx_data_request_ = false;
    bool api_mode_ = false;
    
};

class MotorSocket : public Motor {
 public:
    MotorSocket(std::string interface, std::string address, std::string alias) {
        interface_ = interface;
        address_ = address;
        alias_ = alias;
    }
    virtual ~MotorSocket();

    int create_communication_lock();
    virtual void set_timeout_ms(int timeout_ms) override;
    void open();
    bool connect();
    bool connected() const { return connected_; }

    std::string address() const { return address_; }
    virtual ssize_t read() override;
    virtual ssize_t write() override;

    void rx_data();
    virtual void rx_callback(const uint8_t*, uint16_t) = 0;

    std::string address_;
    std::string interface_;
    std::string alias_;

 protected:
    static const int kProtocolOverheadBytes = 6;

    const static uint32_t RX_BUFFER_SIZE = 2048;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    uint8_t rx_lin_buffer_[RX_BUFFER_SIZE];
    std::atomic<uint32_t> current_read_idx_{0};
    std::thread rx_thread_;
    std::atomic<bool> terminate_{false};
    bool connected_ = false;
    SocketFile realtime_communication_; // relies on parser_
    int fd_communication_lock_;
};

}; // namespace obot
