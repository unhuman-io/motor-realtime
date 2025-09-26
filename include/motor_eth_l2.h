#pragma once
#include "motor_socket.h"

namespace obot {

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 100;

class EthL2RawFile : public SocketFile {
 public:
    EthL2RawFile(std::string address) {
        if (sscanf(address.c_str(), "%*20[^:]:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
            &dst_mac_[0], &dst_mac_[1], &dst_mac_[2],
            &dst_mac_[3], &dst_mac_[4], &dst_mac_[5]) == 6) {
        } else {
            throw RuntimeException("Invalid MAC address format: " + address);
        }
    }
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;
    uint8_t dst_mac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t src_mac_[6] = {0, 0, 0, 0, 0, 0};
};

struct L2Frame {
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint16_t ethertype = htons(0x88B5);
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};

class EthL2CANFile : public EthL2RawFile {
 public:
    EthL2CANFile(std::string address) : EthL2RawFile(address) {
        if (sscanf(address.c_str(), "%*38[^-]-%hhu:%hhu",
            &can_bus_id_, &can_id_) == 2) {
        } else {
            throw RuntimeException("Invalid CAN over MAC address format: " + address);
        }
    }
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;
    uint8_t can_bus_id_ = 0;
    uint8_t can_id_ = 0;
};

struct L2CANFrame {
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint16_t ethertype = htons(0x88B5);
    uint32_t header[6];
    //struct ACFMessage {
        uint8_t acf_message_type;
        uint8_t length;
        uint8_t pad:2;
        uint8_t mtv:1;
        uint8_t rtr:1;
        uint8_t eff:1;
        uint8_t brs:1;
        uint8_t fdf:1;
        uint8_t esi:1;
        uint8_t reserved:3;
        uint8_t can_bus_id:5;
        uint32_t timestamp;
        uint32_t can_id:7;
        uint32_t frame_id:4;
        uint32_t reserved_id:21;
    uint8_t payload[64] = {};
};

enum class EthL2FileMode {
    ETH_L2_RAW,
    ETH_L2_CAN
};

template<EthL2FileMode mode = EthL2FileMode::ETH_L2_RAW>
class MotorEthL2 : public MotorSocket {
 public:
    using EthL2File = std::conditional_t<mode == EthL2FileMode::ETH_L2_RAW, EthL2RawFile,
                      std::conditional_t<mode == EthL2FileMode::ETH_L2_CAN, EthL2CANFile, void>>;
    MotorEthL2(std::string address, std::string address_alias = "") : MotorSocket(get_interface(address), address, address_alias) {
        motor_txt_ = std::move(std::unique_ptr<EthL2File>(new EthL2File(address)));
        EthL2File * motor_txt = static_cast<EthL2File *>(motor_txt_.get());
        open();
        get_interface_mac_address();
        motor_txt->fd_ = fd_;
        motor_txt->fd_communication_lock_ = fd_communication_lock_;
        motor_txt->address_ = address;
        std::memcpy(motor_txt->src_mac_, src_mac_, 6);

        motor_txt->set_api_mode();
        realtime_communication_.fd_ = fd_;
        realtime_communication_.address_ = address;
        realtime_communication_.fd_communication_lock_ = fd_communication_lock_;
        //std::memcpy(realtime_communication_.src_mac_, src_mac_, 6);
        rx_thread_ = std::thread([this]{ this->rx_data(); });

        connected_ = connect();
    }

    static std::string get_interface(std::string_view address) {
        char interface[20];
        if (sscanf(address.data(), "%20[^:]", interface) == 1) {
            return std::string(interface);
        } else {
            throw RuntimeException("Invalid MAC address interface format: " + std::string(address));
        }
    }
    virtual ~MotorEthL2() {}
    void get_interface_mac_address();

    uint8_t src_mac_[6] = {};
    
};

} // namespace obot
