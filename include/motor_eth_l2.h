#pragma once
#include "motor_socket.h"

#include <linux/filter.h>

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

    virtual void set_packet_filter() {
        uint32_t dest_word1;
        std::memcpy(&dest_word1, dst_mac_, 4);
        dest_word1 = htonl(dest_word1);
        uint16_t dest_word2;
        std::memcpy(&dest_word2, dst_mac_+4, 2);
        dest_word2 = htons(dest_word2);
        // Set Berkeley Packet Filter to only receive packets with src_mac == dst_mac_
        struct sock_filter bpf_code[] = {
            // Load first 4 bytes of Ethernet src MAC (offset 6)
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, 0x00000006 }, // BPF_LD+BPF_W+BPF_ABS = 0x20, offset 6
            // Compare with dst_mac_[0..3]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, dest_word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15

            // Load next 2 bytes of Ethernet src MAC (offset 10)
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, 0x0000000A }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 10
            // Compare with dst_mac_[4..5]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 1, dest_word2 }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15

            // Accept packet
            { BPF_RET+BPF_K, 0, 0, 0xFFFFFFFF }, // BPF_RET+BPF_K = 0x06, accept

            // Reject packet
            { BPF_RET+BPF_K, 0, 0, 0 }, // BPF_RET+BPF_K = 0x06, drop
        };

        struct sock_fprog bpf_prog = {
            .len = sizeof(bpf_code)/sizeof(bpf_code[0]),
            .filter = bpf_code,
        };
        if (int result = setsockopt(fd_, SOL_SOCKET, SO_ATTACH_FILTER, &bpf_prog, sizeof(bpf_prog)); result < 0) {
            throw RuntimeException("Failed to set BPF filter " + std::to_string(errno) + ": " + strerror(errno));
        }

    }
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;
    virtual ssize_t _read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    uint8_t dst_mac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t src_mac_[6] = {0, 0, 0, 0, 0, 0};
    uint8_t send_frame_id_ = 1; // command
    uint8_t recv_frame_id_ = 2; // status
    uint8_t send_recv_frame_id_ = 2; // command_status
};

struct L2Frame {
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};

struct L2CANFrame {
    constexpr L2CANFrame() : can_id(0), type(0) {}
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t reserved[8] = {};
    uint16_t can_id:7;
    uint16_t type:4;
    uint8_t length = {};
    uint8_t payload[1000] = {};
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
    virtual void set_packet_filter() {
        uint32_t dest_word1;
        std::memcpy(&dest_word1, dst_mac_, 4);
        dest_word1 = htonl(dest_word1);
        uint16_t dest_word2;
        std::memcpy(&dest_word2, dst_mac_+4, 2);
        dest_word2 = htons(dest_word2);
        // std::cout << "offset dst_mac: " << offsetof(L2CANFrame, src_mac) << std::endl;
        // std::cout << "offset type: " << offsetof(L2CANFrame, type) << std::endl;
        // std::cout << "offset can_id: " << offsetof(L2CANFrame, can_id) << std::endl;
        // Set Berkeley Packet Filter to only receive packets with src_mac == dst_mac_
        struct sock_filter bpf_code[] = {
            // ld [6]
            // jne #0x12345678, drop
            // ldh [0xa]
            // jne #0xabcd, drop
            // ldb [23]
            // jne #1, drop
            // ret #-1
            // drop: ret #0
            // Load first 4 bytes of Ethernet src MAC (offset 6)
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, offsetof(L2CANFrame, src_mac) }, // BPF_LD+BPF_W+BPF_ABS = 0x20, offset 6
            // Compare with dst_mac_[0..3]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 5, dest_word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15

            // Load next 2 bytes of Ethernet src MAC (offset 10)
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, offsetof(L2CANFrame, src_mac) + 4 }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 10
            // Compare with dst_mac_[4..5]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, dest_word2 }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15

            // Load CAN ID
            { BPF_LD+BPF_B+BPF_ABS, 0, 0, offsetof(L2CANFrame, length) - 2 }, // BPF_LD+BPF_B+BPF_ABS = 0x30, offset can_id
            // Compare with can_id_
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 1, can_id_ }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15

            // Accept packet
            { BPF_RET+BPF_K, 0, 0, 0xFFFFFFFF }, // BPF_RET+BPF_K = 0x06, accept

            // Reject packet
            { BPF_RET+BPF_K, 0, 0, 0 }, // BPF_RET+BPF_K = 0x06, drop
        };

        struct sock_fprog bpf_prog = {
            .len = sizeof(bpf_code)/sizeof(bpf_code[0]),
            .filter = bpf_code,
        };
        if (int result = setsockopt(fd_, SOL_SOCKET, SO_ATTACH_FILTER, &bpf_prog, sizeof(bpf_prog)); result < 0) {
            throw RuntimeException("Failed to set BPF filter " + std::to_string(errno) + ": " + strerror(errno));
        }
    }
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) override;
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) override;
    virtual ssize_t _read(char * /* data */, unsigned int /* length */, bool write_read = false) override;
    uint8_t can_bus_id_ = 0;
    uint8_t can_id_ = 0;
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
    MotorEthL2(std::string address, std::string address_alias = "") : MotorSocket(get_interface(address), address, address_alias, new EthL2File(address)) {
        motor_txt_ = std::move(std::unique_ptr<EthL2File>(new EthL2File(address)));
        EthL2File * motor_txt = static_cast<EthL2File *>(motor_txt_.get());
        open();
        get_interface_mac_address();
        motor_txt->fd_ = fd_;
        motor_txt->fd_communication_lock_ = fd_communication_lock_;
        motor_txt->address_ = address;
        motor_txt->send_recv_frame_id_ = 4;
        motor_txt->recv_frame_id_ = 5;
        motor_txt->send_frame_id_ = 4;
        motor_txt->set_packet_filter();
        std::memcpy(motor_txt->src_mac_, src_mac_, 6);
        motor_txt->set_api_mode();

        realtime_communication_->fd_ = fd_;
        realtime_communication_->fd_communication_lock_ = fd_communication_lock_;
        realtime_communication_->address_ = address;
        realtime_communication_->fd_communication_lock_ = fd_communication_lock_;
        realtime_communication_->set_packet_filter();
        std::memcpy(dynamic_cast<EthL2File *>(realtime_communication_)->src_mac_, src_mac_, 6);

        connected_ = connect();
        if constexpr (mode == EthL2FileMode::ETH_L2_CAN) {
            EthL2CANFile * motor_txt_can = static_cast<EthL2CANFile *>(motor_txt_.get());
            devnum_ = motor_txt_can->can_id_;
            dev_path_ += "-can";
            char s[6*3+4] = {};
            std::sprintf(s, "%02x:%02x:%02x:%02x:%02x:%02x-%u",
                motor_txt_can->dst_mac_[0], motor_txt_can->dst_mac_[1], motor_txt_can->dst_mac_[2],
                motor_txt_can->dst_mac_[3], motor_txt_can->dst_mac_[4], motor_txt_can->dst_mac_[5],
                motor_txt_can->can_bus_id_);
            base_path_ = s;
        }
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
