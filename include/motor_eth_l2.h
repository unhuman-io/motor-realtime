#pragma once

#include "motor.h"
#include <string>
#include <vector>

namespace obot {

std::vector<std::string> get_eth_interfaces();

class L2File;

class MotorEthL2 : public Motor {
 public:
    MotorEthL2(std::string address, std::string alias);
    virtual ~MotorEthL2();
    void open();
    bool connected() const { return connected_; }
    virtual ssize_t read() override;
    virtual ssize_t write() override;
    virtual ssize_t aread() override { send_read_request_ = false; return 0; }

    virtual void set_timeout_ms(int timeout_ms) override;
    virtual int get_timeout_ms() const override { return timeout_ms_; }

    static int open_socket(std::string if_name);

 private:
    std::string intf_;
    static uint32_t timeout_ms_;
    bool connected_ = false;
    std::unique_ptr<L2File> realtime_file_;
    bool send_read_request_ = true;
};

}; // namespace obot
