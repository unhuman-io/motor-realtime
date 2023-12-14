#pragma once

#include "motor.h"



//#include <sys/types.h>
//#include <sys/socket.h>
#include <netdb.h>
// #include <arpa/inet.h>

namespace obot {

class UDPFile : public TextFile {
 public:
    struct ObotPacket {
        ObotPacket() : obot{'O', 'B', 'O', 'T'} {}
        char obot[4];
        uint8_t command;
        uint8_t mailbox;
        uint8_t data[1024];
    };
    UDPFile(std::string address) {
        int n = address.find(":");
        if (n == std::string::npos) {
            ip_ = address;
            port_ = "7770";
        } else {
            ip_ = address.substr(0, n);
            port_ = address.substr(n+1,-1);
            if (!port_.size()) {
                port_ = "7770";
            }
        }
        open();
    }
    void open();
    int poll();
    virtual void flush();
    virtual ssize_t read(char * /* data */, unsigned int /* length */);
    virtual ssize_t write(const char * /* data */, unsigned int /* length */);
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */);
    uint8_t send_mailbox_ = 2; // text api
    uint8_t recv_mailbox_ = 1; // text api
    int fd_;
    int timeout_ms_ = 10;
    std::string port_;
    std::string ip_;
 private:
    sockaddr_in addr_ = {};
};

class MotorIP : public Motor {
 public:
    MotorIP(std::string address) : realtime_communication_(address) {
        motor_txt_ = std::move(std::unique_ptr<UDPFile>(new UDPFile(address)));
        realtime_communication_.send_mailbox_ = 4;
        realtime_communication_.recv_mailbox_ = 3;
        fd_ = realtime_communication_.fd_;
        connect();
    }
    
    virtual void set_timeout_ms(int timeout_ms) override;
    void connect();

    std::string ip() const { return realtime_communication_.ip_; }
    uint32_t port() const { return std::atol(realtime_communication_.port_.c_str()); }
    virtual ssize_t read() override;
    virtual ssize_t write() override;

 private:
    UDPFile realtime_communication_;
};

}; // namespace obot