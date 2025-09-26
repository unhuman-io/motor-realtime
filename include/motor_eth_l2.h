#pragma once
#include "motor_socket.h"

namespace obot {

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 100;

class EthL2File : public SocketFile {
 public:
    EthL2File() {}
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;
    uint8_t dst_mac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};

struct L2Frame {
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint16_t ethertype = htons(0x88B5);
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};

class MotorEthL2 : public MotorSocket {
 public:
    MotorEthL2(std::string address, std::string address_alias = "") : alias_(address_alias), MotorSocket(get_interface(address), address) {
        motor_txt_ = std::move(std::unique_ptr<EthL2File>(new EthL2File()));
        EthL2File * motor_txt = static_cast<EthL2File *>(motor_txt_.get());
        open();
        motor_txt->fd_ = fd_;
        motor_txt->fd_communication_lock_ = fd_communication_lock_;
        motor_txt->address_ = address;
        // Convert MAC address string "xx:xx:xx:xx:xx:xx" to uint8_t[6]
        unsigned int mac_bytes[6];
        // Use sscanf to parse MAC address string into uint8_t array
        
            if (sscanf(address.c_str(), "%*20[^:]:%02x:%02x:%02x:%02x:%02x:%02x",
                &mac_bytes[0], &mac_bytes[1], &mac_bytes[2],
                &mac_bytes[3], &mac_bytes[4], &mac_bytes[5]) == 6) {
                for (int i = 0; i < 6; ++i) {
                    motor_txt->dst_mac_[i] = static_cast<uint8_t>(mac_bytes[i]);
                }
            } else {
                throw std::runtime_error("Invalid MAC address format: " + address);
            }

        motor_txt->set_api_mode();
        realtime_communication_.fd_ = fd_;
        realtime_communication_.address_ = address;
        realtime_communication_.fd_communication_lock_ = fd_communication_lock_;
        rx_thread_ = std::thread([this]{ this->rx_data(); });

        connected_ = connect();
    }

    static std::string get_interface(std::string_view address) {
        char interface[20];
        if (sscanf(address.data(), "%20[^:]", interface) == 1) {
            return std::string(interface);
        } else {
            throw std::runtime_error("Invalid MAC address interface format: " + std::string(address));
        }
    }
    virtual ~MotorEthL2() {}
    std::string alias_;
    
};

} // namespace obot
