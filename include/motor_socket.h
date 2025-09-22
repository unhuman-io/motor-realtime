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
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */);

    void set_api_mode() { api_mode_ = true; }
    int lock_communication();
    int unlock_communication();
    int fd_;
    int timeout_ms_ = 50;
    int fd_communication_lock_;
std::string address_;
    void rx_callback(const uint8_t*, uint16_t);
 private:
    ssize_t _read(char * /* data */, unsigned int /* length */, bool write_read = false);
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
    MotorSocket(std::string address, std::string interface = "lo", std::string address_alias = "") {
        address_alias_ = address_alias;
        mac_ = address;
        interface_ = interface;


        
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

    std::string address_;
    std::string address_alias_;
    std::string mac_;
    std::string interface_;

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