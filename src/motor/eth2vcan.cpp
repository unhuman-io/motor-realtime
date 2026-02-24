#include "CLI11.hpp"
#include "exception.h"
#include <cxxabi.h>

#include <unistd.h>
#include <cstring>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <netdb.h>
#include <linux/filter.h>

using namespace obot;

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

int open_vcan(std::string interface) {
    struct sockaddr_can addr;
	struct ifreq ifr;
    int canfd_on = 1;
    int fd;
	if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		throw RuntimeErrnoException("Error opening socket for " + interface);
	}
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on))){
        throw RuntimeErrnoException("Error enabling canfd for " + interface);
    }

    strcpy(ifr.ifr_name, interface.c_str());
    
    if(ioctl(fd, SIOCGIFINDEX, &ifr)) {
        throw RuntimeErrnoException("Error getting ifindex for " + interface);
    }
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        throw RuntimeErrnoException("Error binding " + interface);
	}
    return fd;
}

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
L2Frame l2_frame_out;

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
        throw RuntimeException("Invalid MAC address format");
    }
    return mac;
}

mac_t get_interface_mac_address(int fd, std::string interface) {
    mac_t mac;
    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) == -1) {
        throw RuntimeErrnoException("ioctl SIOCGIFHWADDR failed for " + interface);
    }
    std::memcpy(mac.data(), ifr.ifr_hwaddr.sa_data, 6);
    std::printf("interface mac: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return mac;
}

void set_eth_packet_filter(int fd, mac_t mac, bool src = true) {
        uint32_t word1;
        std::memcpy(&word1, mac.data(), 4);
        word1 = htonl(word1);
        uint16_t word2;
        std::memcpy(&word2, mac.data()+4, 2);
        word2 = htons(word2);
        uint32_t word1_loc = src ? 6 : 0;
        uint32_t word2_loc = word1_loc + 4;
        // Set Berkeley Packet Filter to only receive packets with mac matching
        struct sock_filter bpf_code[] = {
            // Load first 4 bytes of Ethernet MAC
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, word1_loc }, // BPF_LD+BPF_W+BPF_ABS = 0x20, offset 6
            // Compare with dst_mac_[0..3]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 5, word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Load next 2 bytes of Ethernet MAC
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, word2_loc }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 10
            // Compare with dst_mac_[4..5]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, word2 }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Check first byte of payload accept zero
            { BPF_LD+BPF_B+BPF_ABS, 0, 0, 14 },
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 1, 0 },
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

int open_eth(std::string interface, std::string mac_address, bool gateway_mode, std::string gateway_mac) {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) {
      throw RuntimeErrnoException("socket failed for " + interface);
    }
    sockaddr_ll server_addr = {};
    server_addr.sll_family = AF_PACKET;
    server_addr.sll_protocol = htons(0x88B5);
    server_addr.sll_ifindex = if_nametoindex(interface.c_str());

    int retval = bind(fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (retval < 0) {
      throw RuntimeErrnoException("bind failed for " + interface);
    }

    mac_t * this_mac = !gateway_mode ? &l2_frame_out.src_mac : &l2_frame_out.dst_mac;
    mac_t * dst_mac = !gateway_mode ? &l2_frame_out.dst_mac : &l2_frame_out.src_mac;
    *dst_mac = str2mac(mac_address);

    if (gateway_mode) {
        *this_mac = str2mac(gateway_mac);
    } else {
        *this_mac = get_interface_mac_address(fd, interface);
    }
    set_eth_packet_filter(fd, *dst_mac, !gateway_mode);
    return fd;
}

std::vector<Payload> parse_eth_payload(uint8_t *frame_payload, ssize_t length) {
    std::vector<Payload> payloads;
    int ptr = 0;
    while (ptr < length-(PAYLOAD_HEADER_SIZE-1)) {
        Payload payload {};
        std::memcpy(&payload, &frame_payload[ptr], PAYLOAD_HEADER_SIZE);
        ptr += PAYLOAD_HEADER_SIZE;
        payload.topic_id = ntohs(payload.topic_id);
        if (payload.topic_id == 0) {
            break;
        }
        payload.length = ntohs(payload.length);
        if (payload.length > MAX_PAYLOAD_LENGTH) {
            throw RuntimeHexDumpException("payload length: " + std::to_string(payload.length) +
                ", max allowed: " + std::to_string(MAX_PAYLOAD_LENGTH), frame_payload, length);
        }
        if (payload.length > 0) {
            payload.data = &frame_payload[ptr];
        }
        ptr += payload.length;
        payloads.push_back(payload);
        std::cout << "\ttopic_id: " << std::hex << payload.topic_id << std::dec << ", length: " << (int) payload.length << std::endl;
    }
    std::cout << "\t" << payloads.size() << " payloads" << std::endl;
    if (ptr > length) {
        throw RuntimeHexDumpException("payload sum error, " + std::to_string(payloads.size()) +
            " messages, total length " + std::to_string(ptr), frame_payload, length);
    }
    return payloads;
}


int _main(int argc, char** argv) {
    std::string mac_address {"00:00:00:00:00:00"};
    std::string vcan_interface {"vcan0"};
    std::string interface {"lo"};
    std::string gateway_mac {"00:00:00:00:00:00"};
    CLI::App app{"Utility for converting ethernet l2 communication to vcan\n"
                 "\n"
                 "Example:\n"
                 "sudo modprobe vcan\n"
                 "sudo ip link add dev vcan0 type vcan\n"
                 "sudo ip link set up vcan0\n"
                 "eth2vcan -i eth0 -m 12:34:56:78:ab:cd\n"
                 "# another terminal\n"
                 "motor_util -f vcan0"};
    app.add_option("-v,--vcan", vcan_interface, "Use VCAN_INTERFACE for vcan")->type_name("VCAN_INTERFACE")->capture_default_str()->expected(1);
    app.add_option("-m,--mac", mac_address, "Use MAC address MAC_ADDRESS")->type_name("MAC_ADDRESS")->capture_default_str()->expected(1);
    app.add_option("-i,--interface", interface, "Use network interface INTERFACE")->type_name("INTERFACE")->capture_default_str()->expected(1);
    auto gateway_option = app.add_option("-g,--gateway", gateway_mac, "Gateway mode for converting can to ethernet destination at MAC_ADDRESS")->type_name("MAC_ADDRESS")->capture_default_str()->expected(0,1);
    CLI11_PARSE(app, argc, argv);

    int fd_vcan = open_vcan(vcan_interface);
    int fd_eth = open_eth(interface, mac_address, static_cast<bool>(*gateway_option), gateway_mac);

    pollfd poll_fds[2];
    poll_fds[0].fd = fd_vcan;
    poll_fds[0].events = POLLIN;
    poll_fds[1].fd = fd_eth;
    poll_fds[1].events = POLLIN;
    while(1) {
        int poll_result = poll(poll_fds, 2, 1);
        if (poll_result < 0) {
            throw RuntimeErrnoException("Poll error");
        } else if (poll_result > 0) {
            if (poll_fds[0].revents) {
                canfd_frame can_frame;
                int nbytes = ::read(fd_vcan, &can_frame, sizeof(canfd_frame));
                if (nbytes <= 0) {
                    throw RuntimeErrnoException("vcan read error");
                }
                std::cout << "can nbytes " << nbytes << std::endl;
                uint16_t length = can_frame.len;
                Payload payload {
                    .topic_id = htons(can_frame.can_id),
                    .length = htons(length),
                    .data = can_frame.data
                };
                std::memset(&l2_frame_out.payload, 0, 64-L2_HEADER_SIZE);
                std::memcpy(l2_frame_out.payload, &payload, PAYLOAD_HEADER_SIZE);
                std::memcpy(l2_frame_out.payload+PAYLOAD_HEADER_SIZE, payload.data, length);
                int length_out = std::max(length+PAYLOAD_HEADER_SIZE+L2_HEADER_SIZE, 64);
                int result = send(fd_eth, &l2_frame_out, length_out, 0);
                if (result < 0) {
                    std::cout << "fd eth " << fd_eth << std::endl;
                    std::cout << RuntimeHexDumpException::hex_dump((uint8_t *) &l2_frame_out, length_out) << std::endl;
                    throw RuntimeErrnoException("eth write error");
                }
            }
            if (poll_fds[1].revents) {
                L2Frame frame {};
                int nbytes = ::read(fd_eth, &frame, sizeof(frame));
                if (nbytes <= 0) {
                    throw RuntimeErrnoException("eth read error");
                }
                std::cout << "eth nbytes " << nbytes << std::endl;
                for (auto &payload : parse_eth_payload(frame.payload, nbytes-L2_HEADER_SIZE)) {
                    uint8_t length_uint8 {static_cast<uint8_t>(payload.length)};
                    canfd_frame frame_out {
                        .can_id = payload.topic_id,
                        .len = length_uint8,
                    };
                    std::memcpy(frame_out.data, payload.data, length_uint8);
                    
                    int result = send(fd_vcan, &frame_out, sizeof(canfd_frame), 0);
                    if (result < 0) {
                        throw RuntimeErrnoException("vcan write error");
                    }
                }
            }
        }
    }

    return 0;
}

int main(int argc, char** argv) {
    try {
        return _main(argc, argv);
    } catch (const RuntimeException &e) {
        std::cerr << "Caught RuntimeException" << std::endl;
        std::cerr << " what(): " << e.what() << std::endl;
        std::cerr << e.location_print() << std::endl;    
    } catch (const std::exception &e) {
        int status;
        std::cerr << "Caught exception of type " << abi::__cxa_demangle(typeid(e).name(), NULL, NULL, &status) << std::endl;
        std::cerr << "  what():  " << e.what() << std::endl;
        return 1;
    }
}
