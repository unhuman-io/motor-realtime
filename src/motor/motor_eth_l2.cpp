#include "motor_eth_l2.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <linux/filter.h>
#include <net/if_arp.h>

#include <ifaddrs.h>
#include <array>
#include <sstream>
#include <iomanip>

#include "poll.h"

namespace obot {

class RuntimeHexDumpException : public RuntimeException {
  public:
    RuntimeHexDumpException(const std::string& msg, uint8_t *data_ptr, int length) :
        RuntimeException(
            msg + "\n" + hex_dump(data_ptr, length)
        ) {}
    static std::string hex_dump(uint8_t *data_ptr, int length) {
        std::stringstream stream;
        stream << "    hex dump length: " << length;
        stream << std::hex << std::setfill('0') << std::setw(2);
        for (int i = 0; i < length; i++ ) {
            if (i % 16 == 0) {
                stream << "\n    ";
            }
            stream << std::setw(2) << (int) data_ptr[i] << " ";
            
        }
        return stream.str();
    }
};

enum class L2MessageType
{
    OBOT_CMD = 0x01,
    OBOT_CMD_STATUS = 0x02,
    OBOT_STATUS = 0x03,
    OBOT_ASCII_CMD = 0x04,
    OBOT_ASCII_RESPONSE = 0x05,
    OBOT_ENUM = 0xF,
};

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 3000;
constexpr int MAX_PAYLOAD_LENGTH = 64;
using mac_t = std::array<uint8_t, 6>;
struct L2Frame {
    mac_t dst_mac = {};
    mac_t src_mac = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t reserved[6] = {};
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};
constexpr int L2_HEADER_SIZE = sizeof(L2Frame) - sizeof(L2Frame::payload);
static_assert(L2_HEADER_SIZE == 20);

struct Payload {
    uint16_t topic_id;
    uint16_t length;
    uint8_t* data;
};
constexpr int PAYLOAD_HEADER_SIZE = sizeof(Payload::topic_id) + sizeof(Payload::length);
static_assert(PAYLOAD_HEADER_SIZE == 4);

struct TopicId {
    uint16_t node_id:4;
    uint16_t bus_id:3;
    uint16_t type:4;
};

mac_t str2mac(std::string mac_str) {
    mac_t mac;
    if (sscanf(mac_str.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &mac[0], &mac[1], &mac[2],
        &mac[3], &mac[4], &mac[5]) == 6) {
    } else {
        throw RuntimeException("Invalid MAC address format (" + mac_str + ")");
    }
    return mac;
}

std::string mac2str(const mac_t &mac) {
    char hex_string[18];
    snprintf(hex_string, sizeof(hex_string), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return hex_string;
}

mac_t get_interface_mac_address(int fd, std::string interface) {
    mac_t mac {};
    if (interface != "any" ) {
        struct ifreq ifr = {};
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd, SIOCGIFHWADDR, &ifr) == -1) {
            throw RuntimeErrnoException("ioctl SIOCGIFHWADDR failed for " + interface);
        }
        std::memcpy(mac.data(), ifr.ifr_hwaddr.sa_data, 6);
    }
    return mac;
}

static std::vector<std::string> get_eth_interfaces() {
    std::vector<std::string> interfaces;
    struct ifaddrs *addrs,*tmp;

    if (getifaddrs(&addrs)) {
        throw RuntimeException("Error getting interfaces: " + std::to_string(errno) + ": " + strerror(errno));
    }
    tmp = addrs;

    while (tmp) {
        //std::cout << "interface: " << tmp->ifa_name << " flags " << tmp->ifa_flags << std::endl;
        // test if interface is ethernet
        if (tmp->ifa_flags & IFF_UP) {
            struct sockaddr_ll *sll = (struct sockaddr_ll *)tmp->ifa_addr;
            if (sll && sll->sll_hatype == ARPHRD_ETHER) {
                //printf("Interface %s is Ethernet-compatible\n", tmp->ifa_name);
                interfaces.push_back(tmp->ifa_name);
            }
        }
        tmp = tmp->ifa_next;
    }
    if (interfaces.empty()) {
        throw RuntimeException("No valid ETH interfaces found");
    } else {
        // std::cout << "Found ETH interfaces: ";
        // for (auto &s : interfaces) {
        //     std::cout << s << " ";
        // }
        // std::cout << std::endl;
    }
    freeifaddrs(addrs);
    return interfaces;
}

class L2File : public TextFile {
 public:
    L2File(std::string interface, mac_t dst_mac, L2MessageType cmd_type, L2MessageType cmd_status_type, L2MessageType status_type) : 
        cmd_type_(cmd_type), cmd_status_type_(cmd_status_type), status_type_(status_type),
        dst_mac_(dst_mac), interface_(interface) {
        

        // lock file to prevent multiple instances at the same time
        lock_file_ = "/tmp/obot." + interface_ + "-" + mac2str(dst_mac) + ".lock";
        fd_lock_ = ::open(lock_file_.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_lock_ < 0) {
            throw RuntimeException("Error opening lock file " + lock_file_ + ":" + std::to_string(errno) + ": " + strerror(errno));
        }
        int err = ::lseek(fd_lock_, 0, SEEK_SET);
        if (err < 0) {
            throw RuntimeException("Error lseek lock file " + lock_file_ + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
        open();
    }

    void open() {
        fd_ = ::socket(AF_PACKET, SOCK_RAW, htons(0x88b5));
        if (fd_ < 0) {
        throw RuntimeErrnoException("socket failed for " + interface_);
        }

        // if (interface_ != "any") {
        //     if (setsockopt(fd_, SOL_SOCKET, SO_BINDTODEVICE, interface_.c_str(), interface_.size()) < 0) {
        //         throw RuntimeErrnoException("setsockopt SO_BINDTODEVICE error");
        //     }
        // }

        node_id_ = dst_mac_[5];
        TopicId topic_id {
            .node_id = node_id_ ,
            .type = static_cast<uint16_t>(status_type_)
        };
        uint16_t topic_id_uint;
        std::memcpy(&topic_id_uint, &topic_id, sizeof(topic_id_uint));

        set_eth_packet_filter(fd_, dst_mac_, htons(topic_id_uint));
        flush();
                // still need to bind in order to send, I guess
        sockaddr_ll server_addr = {};
        server_addr.sll_family = AF_PACKET;
        server_addr.sll_protocol = htons(0x88B5);
        server_addr.sll_ifindex = if_nametoindex(interface_.c_str());
    
        int retval = bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (retval < 0) {
            throw RuntimeErrnoException("bind failed for " + interface_);
        }

        std::memcpy(&l2_frame_out_.dst_mac, &dst_mac_, sizeof(dst_mac_));
        mac_t src_mac = get_interface_mac_address(fd_, interface_);
        std::memcpy(&l2_frame_out_.src_mac, &src_mac, sizeof(src_mac));

        
    }

    void close() {
        ::close(fd_);
    }

    void flush() {
        // flush frames received before filter
        pollfd poll_fd {
            .fd = fd_,
            .events = POLLIN
        };
        
        for (int result = ::poll(&poll_fd, 1, 0); result > 0; result = ::poll(&poll_fd, 1, 0)) {
            if (result < 0) {
                throw RuntimeErrnoException("poll error during flush");
            }
            char buf[MAX_ETH_L2_PAYLOAD_SIZE];
            ::read(fd_, buf, MAX_ETH_L2_PAYLOAD_SIZE);
        }
    }

    void set_eth_packet_filter(int fd, mac_t mac, uint16_t topic_id) {
        uint32_t word1;
        std::memcpy(&word1, mac.data(), 4);
        word1 = htonl(word1);
        uint16_t word2;
        std::memcpy(&word2, mac.data()+4, 2);
        word2 = htons(word2);
        // Set Berkeley Packet Filter to only receive packets with mac matching
        struct sock_filter bpf_code[] = {
            // Load first 4 bytes of Ethernet MAC
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, 6 }, // BPF_LD+BPF_W+BPF_ABS = 0x20, offset 6
            // Compare with dst_mac_[0..3]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 7, word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Load next 2 bytes of Ethernet MAC
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, 10 }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 10
            // Compare with dst_mac_[4..5]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 5, word2 }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Check first 4 bytes of payload accept zero
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, 14 },
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, 0 },
            // Check packet type field
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, L2_HEADER_SIZE },
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 1, htons(topic_id) },

            // Accept packet
            { BPF_RET+BPF_K, 0, 0, 0xFFFFFFFF }, // BPF_RET+BPF_K = 0x06, accept
            // Reject packet
            { BPF_RET+BPF_K, 0, 0, 0 }, // BPF_RET+BPF_K = 0x06, drop
        };

        struct sock_fprog bpf_prog = {
            .len = sizeof(bpf_code)/sizeof(bpf_code[0]),
            .filter = bpf_code,
        };
        if (int result = setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &bpf_prog, sizeof(bpf_prog)); result < 0) {
            throw RuntimeErrnoException("Failed to set BPF filter");
        }
    }

    // use a lock file to provide exclusive access to the device during a 
    // write followed by read interface to the text api
    int lock() {
        int err = lockf(fd_lock_, F_LOCK, 1);
        if (err) {
            std::cerr << "error locking " + lock_file_;
            pid_t pid;
            int err2 = get_lock_pid(fd_lock_, &pid);
            if (err2 == 0) {
                std::cerr << ", already locked by process: " << pid;
            }
            std::cerr << std::endl;
        }
        return err;
    }

    int unlock() {
        // close();
        int err = lockf(fd_lock_, F_ULOCK, 0);
        if (err) {
            std::cerr << "error unlocking " + lock_file_;
            pid_t pid;
            int err2 = get_lock_pid(fd_lock_, &pid);
            if (err2 == 0) {
                std::cerr << ", locked by process: " << pid;
            }
            std::cerr << std::endl;
        }
        return err;
    }

    ssize_t _read(char * data, unsigned int length, bool request = false) {
        if (request) {
            TopicId topic_id {
                .node_id = node_id_ ,
                .type = static_cast<uint16_t>(status_type_)
            };
            uint16_t topic_id_uint;
            std::memcpy(&topic_id_uint, &topic_id, sizeof(topic_id));
            Payload payload {
                .topic_id = htons(topic_id_uint),
            };
            std::memset(&l2_frame_out_.payload, 0, 64-L2_HEADER_SIZE);
            std::memcpy(l2_frame_out_.payload, &payload, PAYLOAD_HEADER_SIZE);
            int length_out = 64;
            int result = send(fd_, &l2_frame_out_, length_out, 0);
            if (result < 0) {
                std::cout << RuntimeHexDumpException::hex_dump((uint8_t *) &l2_frame_out_, length_out) << std::endl;
                throw RuntimeErrnoException("eth write error");
            }
        }
        pollfd poll_fd {
            .fd = fd_,
            .events = POLLIN
        };
        if (int poll_result = poll(&poll_fd, 1, timeout_ms_); poll_result < 0) {
            throw RuntimeErrnoException("poll error in read");
        } else if (poll_result == 0) {
            throw RuntimeException("poll timeout in read");
        }

        L2Frame frame {};
        int nbytes = ::read(fd_, &frame, sizeof(frame));
        if (nbytes <= 0) {
            throw RuntimeErrnoException("eth read error");
        }
    
        Payload payload {};
        std::memcpy(&payload, frame.payload, PAYLOAD_HEADER_SIZE);
        payload.topic_id = ntohs(payload.topic_id);
        payload.length = ntohs(payload.length);
        if (payload.length > MAX_PAYLOAD_LENGTH || payload.length > nbytes - L2_HEADER_SIZE - PAYLOAD_HEADER_SIZE) {
            throw RuntimeHexDumpException("payload length error: " + std::to_string(payload.length), reinterpret_cast<uint8_t*>(&frame), nbytes);
        }
        length = std::min(static_cast<unsigned int>(payload.length), length);
        std::memcpy(data, &frame.payload[PAYLOAD_HEADER_SIZE], length);
        return length;
    }

    virtual ssize_t read(char * data, unsigned int length) {
        ssize_t retval = _read(data, length);

        if (retval >= sizeof(APIControlPacket) && data[0] == 0) {
            // a control packet
            APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data);
            if (packet->type == TIMEOUT_REQUEST) {
                // timeout request
                if (retval == sizeof(APIControlPacket)) {
                    // timeout request
                    // retriggers the read with the new timeout
                    uint32_t old_timeout_ms = timeout_ms_;
                    timeout_ms_ += packet->timeout_request.timeout_us/1000;
                    ssize_t retval = read(data, length);
                    timeout_ms_ = old_timeout_ms;
                    return retval;
                }
            } else if (packet->type == LONG_PACKET) {
                // long packet
                uint16_t total_length = packet->long_packet.total_length;
                uint16_t packet_number = packet->long_packet.packet_number;
                const uint8_t header_size = sizeof(APIControlPacket);
                uint16_t total_count_received = retval - header_size;
                if (total_length > length) {
                    // too long
                    return -EINVAL;
                }
                std::memmove(data, data + header_size, total_count_received);
                while (total_length > total_count_received) {
                    // assemble multiple packets
                    char * data_ptr = data + total_count_received;
                    retval = _read(data_ptr, length);
                    if (retval < 0) {
                        return retval;
                    }
                    APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data_ptr);
                    if (packet->type != LONG_PACKET) {
                        std::cerr << "Error: expected long packet, got " << packet->type << std::endl;
                        return -EINVAL;
                    }
                    if (packet->long_packet.packet_number != ++packet_number) {
                        std::cerr << "Error: expected packet number " << packet_number << ", got " << packet->long_packet.packet_number << std::endl;
                        return -EINVAL;
                    }
                    //std::cout << packet->long_packet.packet_number << retval << std::endl;
                    total_count_received += retval - header_size;
                    std::memmove(data_ptr, data_ptr + header_size, retval - header_size);
                }
                if (total_count_received < total_length) {
                    std::cerr << "Error: expected " << total_length << " bytes, got " << total_count_received << std::endl;
                    return -EINVAL;
                }
                retval = total_count_received;
            }
        }
        unlock();
        return retval;
    }


    virtual ssize_t write(const char * data, unsigned int length, bool writeread = false) {
        lock();
        L2MessageType cmd_type = writeread ? cmd_status_type_ : cmd_type_;
        TopicId topic_id {
            .node_id = node_id_ ,
            .type = static_cast<uint16_t>(cmd_type)
        };
        uint16_t topic_id_uint;
        std::memcpy(&topic_id_uint, &topic_id, sizeof(topic_id));
        Payload payload {
            .topic_id = htons(topic_id_uint),
            .length = htons(length),
        };
        std::memset(&l2_frame_out_.payload, 0, 64-L2_HEADER_SIZE);
        std::memcpy(l2_frame_out_.payload, &payload, PAYLOAD_HEADER_SIZE);
        length = std::min(length, static_cast<unsigned int>(MAX_ETH_L2_PAYLOAD_SIZE));
        std::memcpy(l2_frame_out_.payload+PAYLOAD_HEADER_SIZE, data, length);
        int length_out = std::max(length+PAYLOAD_HEADER_SIZE+L2_HEADER_SIZE, 64u);
        int result = send(fd_, &l2_frame_out_, length_out, 0);
        if (result < 0) {
            std::cout << RuntimeHexDumpException::hex_dump((uint8_t *) &l2_frame_out_, length_out) << std::endl;
            throw RuntimeErrnoException("eth write error");
        }
        return length_out;
    }

    virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
        //int err = lock();
        // if (err) {
        //     return err;
        // }
        ssize_t nbytes = write(data_out, length_out, true);
        if (nbytes < 0) {
            return nbytes;
        }
        int retval = read(data_in, length_in);
        //err = unlock();
        // if (err) {
        //     return err;
        // }
        return retval;
    }

    int fd_;
    L2MessageType cmd_type_, cmd_status_type_, status_type_;
    std::string interface_;
    mac_t dst_mac_;
    int timeout_ms_ = 10;
    int fd_lock_;
    L2Frame l2_frame_out_;
    uint8_t node_id_;
    
    std::string lock_file_;
};

MotorEthL2::MotorEthL2(std::string address, std::string alias) {
    std::string interface;
    std::string mac;
    if (int n = address.find("-"); n == std::string::npos) {
        interface = "any";
        mac = address;
    } else {
        interface = address.substr(0, n);
        mac = address.substr(n+1,-1);
    }
    mac_t dst_mac = str2mac(mac);
    devnum_ = dst_mac[5];
    dev_path_ = mac;

    if (interface == "any") {
        auto interfaces = get_eth_interfaces();
        for (auto & tmp_interface : interfaces) {
            auto motor_txt = L2File(tmp_interface, dst_mac, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_RESPONSE);
            try {
                std::string str_out = "messages_version";
                char str_in[64];
                motor_txt.writeread(str_out.c_str(), str_out.length(), str_in, 64);
                interface = tmp_interface;
                break;
            } catch (RuntimeException &e) {}
        }
        if (interface == "any") {
            throw RuntimeException(mac + " not found");
        }
    }

    base_path_ = interface;

    realtime_file_ = std::unique_ptr<L2File>(new L2File(interface, dst_mac, L2MessageType::OBOT_CMD, L2MessageType::OBOT_CMD_STATUS, L2MessageType::OBOT_STATUS));
    open();

    motor_txt_ = std::move(std::unique_ptr<L2File>(new L2File(interface, dst_mac, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_RESPONSE)));
	
    messages_version_ = operator[]("messages_version").get();
    name_ = operator[]("name").get();
    version_ = operator[]("version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
    serial_number_ = operator[]("serial").get();
    connected_ = true;
}

MotorEthL2::~MotorEthL2() {}

uint32_t MotorEthL2::timeout_ms_ = 10;
void MotorEthL2::set_timeout_ms(int timeout_ms) {
    timeout_ms_ = timeout_ms;
    static_cast<L2File*>(motor_txt_.get())->timeout_ms_ = timeout_ms;
}

void MotorEthL2::open() {
    
}


ssize_t MotorEthL2::read() {
    L2Frame frame_in;
    int nbytes = 0;
    if (int retval = realtime_file_->_read(reinterpret_cast<char *>(&frame_in), sizeof(frame_in), true);
        retval < 0) {
        throw RuntimeErrnoException("Error on EthL2 read");
    } else {
        nbytes = std::min(retval, static_cast<int>(sizeof(status_)));
        std::memcpy(&status_, &frame_in, nbytes);
    }
    return nbytes;
}

ssize_t MotorEthL2::write() {
    int nbytes = 0;
    if (int retval = realtime_file_->write(reinterpret_cast<const char*>(&command_), sizeof(command_));
        retval < 0) {
        throw RuntimeErrnoException("Error on EthL2 write");
    } else {
        nbytes = retval;
    }
    return nbytes;
}



// std::vector<std::string> MotorCAN::enumerate_can_devices(std::string interface) {
//     std::vector<std::string> devices;
//     std::vector<std::string> interfaces;
//     if (interface == "any") {
//         interfaces = get_can_interfaces();
//     } else {
//         interfaces.push_back(interface);
//     }

//     int fd = open_socket(interface);
//     struct can_filter rfilter[1];
//     rfilter[0].can_id   = 0x780;
//     rfilter[0].can_mask = 0x780 | CAN_EFF_FLAG | CAN_RTR_FLAG;

//     if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))) {
//         throw RuntimeException("Error setting filter for " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
//     }


//     for (std::string &interface : interfaces) {
//         int write_fd = open_socket(interface);

//         struct canfd_frame frame = {};
//         frame.can_id  = 0xf << 7 | 0x7f | CAN_RTR_FLAG;
//         frame.len = 0;

//         int nbytes = ::write(write_fd, &frame, sizeof(struct canfd_frame));
//         if (nbytes < 0) {
//             throw RuntimeException("Error writing can " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
//         }
//     }

//     pollfd tmp;
//     tmp.fd = fd;
//     tmp.events = POLLIN;
//     Timer t(timeout_ms_ * 1000 * 1000); // 10 ms
//     do {
//         struct timespec timeout = {};
//         timeout.tv_nsec = t.get_time_remaining_ns();
//         if (timeout.tv_nsec == 0) {
//             break;
//         }
//         int poll_result = ::ppoll(&tmp, 1, &timeout, nullptr /*sigmask*/);
//         if (poll_result > 0) {
//             struct canfd_frame frame;
//             struct sockaddr_can addr;
//             socklen_t len = sizeof(addr);
//             int nbytes = recvfrom(fd, &frame, sizeof(struct can_frame),
//                   0, (struct sockaddr*)&addr, &len);
//             struct ifreq ifr = {};
//             ifr.ifr_ifindex = addr.can_ifindex;
//             ioctl(fd, SIOCGIFNAME, &ifr);
//             if (nbytes >= 0) {
//                 int devnum = frame.can_id & 0x7F;
//                 devices.push_back(std::string(ifr.ifr_name) + ":" + std::to_string(devnum));
//             } else {
//                 throw RuntimeException("Error reading " + interface + "(" + std::string(ifr.ifr_name) + ")" ": " + std::to_string(errno) + ": " + strerror(errno));
//             }
//         } else if (poll_result < 0) {
//             throw RuntimeException("Error polling " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
//         }
//     } while (t.get_time_remaining_ns() > 0);

//     return devices;
// }

}; // namespace obot
