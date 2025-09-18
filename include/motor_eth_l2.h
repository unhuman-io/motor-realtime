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
    MotorEthL2(std::string address, std::string address_alias = "") : MotorSocket(address, address_alias) {
        SocketFile * motor_txt = static_cast<SocketFile *>(motor_txt_.get());
    }
    virtual ~MotorEthL2() {}
};

} // namespace obot
