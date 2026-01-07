#pragma once

#include "motor.h"

#include <netdb.h>
#include <atomic>

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
    virtual void set_packet_filter() = 0;
    
    void set_api_mode() { api_mode_ = true; }
    int lock_communication();
    int unlock_communication();
    int fd_;
    int timeout_ms_ = 50;
    int fd_communication_lock_;
    std::atomic<int> communication_lock_count_{};
    std::string address_;
 protected:
    virtual ssize_t _read(char * /* data */, unsigned int /* length */, bool write_read = false);
 private:
    static constexpr int RX_BUFFER_SIZE = 1000;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    bool api_mode_ = false;
    
};

class MotorSocket : public Motor {
 public:
    MotorSocket(std::string interface, std::string address, std::string alias, SocketFile *realtime_communication = nullptr) : Motor(), realtime_communication_(realtime_communication) {
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

    std::string address_;
    std::string interface_;
    std::string alias_;

 protected:
    bool connected_ = false;
    SocketFile *realtime_communication_;
    int fd_communication_lock_;
};

}; // namespace obot
