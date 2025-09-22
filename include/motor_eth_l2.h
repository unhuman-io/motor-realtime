#pragma once
#include "motor_socket.h"

namespace obot {

class EthL2File : public SocketFile {
 public:
    EthL2File() {}
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;
};

class MotorEthL2 : public MotorSocket {
 public:
    MotorEthL2(std::string address, std::string address_alias = "") : alias_(address_alias), MotorSocket(address, address_alias) {
        motor_txt_ = std::move(std::unique_ptr<EthL2File>(new EthL2File()));
        EthL2File * motor_txt = static_cast<EthL2File *>(motor_txt_.get());
        open();
        motor_txt->fd_ = fd_;
        motor_txt->fd_communication_lock_ = fd_communication_lock_;
        motor_txt->address_ = address;
        motor_txt->set_api_mode();
        realtime_communication_.fd_ = fd_;
        realtime_communication_.address_ = address;
        realtime_communication_.fd_communication_lock_ = fd_communication_lock_;
        rx_thread_ = std::thread([this]{ this->rx_data(); });

        connected_ = connect();
    }
    virtual ~MotorEthL2() {}
    std::string alias_;
};

} // namespace obot
