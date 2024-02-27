#pragma once

#include "motor.h"

#include <netdb.h>
#include "protocol_parser.h"
#include <thread>
#include <atomic>
#include <condition_variable>

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
    UDPFile(figure::ProtocolParser &parser) :
        parser_(parser) {
        register_parser_callbacks();
    }
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

    void register_parser_callbacks() {
        parser_.registerCallback(recv_frame_id_, [this](const uint8_t* buf, uint16_t len){ 
            this->rx_callback(buf, len); });
        parser_.registerCallback(send_recv_frame_id_, [this](const uint8_t* buf, uint16_t len){ 
            this->rx_callback(buf, len); });
    }
    void rx_callback(const uint8_t*, uint16_t);
    sockaddr_in addr_ = {};
 private:
    figure::ProtocolParser &parser_;
    std::condition_variable rx_data_cv_;
    std::mutex rx_data_cv_m_;
    uint8_t rx_buf_[1024];
};

class MotorIP : public Motor {
 public:
    MotorIP(std::string address) : realtime_communication_(parser_) {

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

        motor_txt_ = std::move(std::unique_ptr<UDPFile>(new UDPFile(parser_)));
        UDPFile * motor_txt = static_cast<UDPFile *>(motor_txt_.get());
        motor_txt->send_recv_frame_id_ = 4;
        motor_txt->recv_frame_id_ = 5;
        motor_txt->send_frame_id_ = 4;
        motor_txt->register_parser_callbacks();
        realtime_communication_.register_parser_callbacks();
        open();
        motor_txt->fd_ = fd_;
        motor_txt->addr_ = addr_;
        realtime_communication_.fd_ = fd_;
        realtime_communication_.addr_ = addr_;
        rx_thread_ = std::thread([this]{ this->rx_data(); });
        connect();
    }
    
    virtual void set_timeout_ms(int timeout_ms) override;
    void open();
    void connect();

    std::string ip() const { return ip_; }
    uint32_t port() const { return std::atol(port_.c_str()); }
    virtual ssize_t read() override;
    virtual ssize_t write() override;

    void rx_data();

    std::string port_;
    std::string ip_;
    std::string addrstr_;
    sockaddr_in addr_ = {};

 private:
    UDPFile realtime_communication_;
    static const int kProtocolOverheadBytes = 6;
    UDPFile::ObotPacket read_buffer_;
    UDPFile::ObotPacket write_buffer_;

    const static uint32_t RX_BUFFER_SIZE = 2048;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    uint8_t rx_lin_buffer_[RX_BUFFER_SIZE];
    figure::ProtocolParser parser_{rx_buffer_, RX_BUFFER_SIZE};
    std::atomic<uint32_t> current_read_idx_{0};
    std::thread rx_thread_;
};

}; // namespace obot