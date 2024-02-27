#pragma once

#include "motor.h"

#include <netdb.h>

namespace obot {

class UDPFile : public TextFile {
 public:
    /*
    * [ Start bytes (2) ] = {0xCA, 0xFE}
    * [ Frame ID (1) ]
    * [ Payload Length (1) ]
    * [ Payload (n) ]
    * [ CRC (2) ]
    */
    struct ObotPacket {
        ObotPacket() : start_bytes{0xCA, 0xFE} {}
        uint8_t start_bytes[2];
        uint8_t frame_id;
        uint8_t length;
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
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false);
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false);
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */);
    uint8_t send_frame_id_ = 1; // command
    uint8_t recv_frame_id_ = 2; // status
    uint8_t send_recv_frame_id_ = 3; // command_status
    int fd_;
    int timeout_ms_ = 10;
    std::string port_;
    std::string ip_;
    std::string addrstr_;
 private:
    sockaddr_in addr_ = {};
};

class MotorIP : public Motor {
 public:
    MotorIP(std::string address) : realtime_communication_(address) {
        motor_txt_ = std::move(std::unique_ptr<UDPFile>(new UDPFile(address)));
        UDPFile * motor_txt = static_cast<UDPFile *>(motor_txt_.get());
        motor_txt->send_recv_frame_id_ = 4;
        motor_txt->recv_frame_id_ = 5;
        motor_txt->send_frame_id_ = 4;
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
    static const int kProtocolOverheadBytes = 6;
    UDPFile::ObotPacket read_buffer_;
    UDPFile::ObotPacket write_buffer_;
};

}; // namespace obot