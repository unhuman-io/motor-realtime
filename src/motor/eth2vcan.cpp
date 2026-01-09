#include "CLI11.hpp"
#include "exception.h"

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

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 1000;
using mac_t = std::array<uint8_t, 6>;
struct L2Frame {
    mac_t dst_mac = {};
    mac_t src_mac = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t reserved[8] = {};
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};
L2Frame l2_frame_out;

struct Payload {
    uint16_t topic_id;
    uint8_t length;
    uint8_t* data;
};

struct TopicId {
    uint16_t node_id:4;
    uint16_t bus_id:3;
    uint16_t type:4;
};

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
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Load next 2 bytes of Ethernet MAC
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, word2_loc }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 10
            // Compare with dst_mac_[4..5]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 1, word2 }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
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

int open_eth(std::string interface, std::string mac_address, bool gateway_mode) {
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
    if (sscanf(mac_address.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &(*dst_mac)[0], &(*dst_mac)[1], &(*dst_mac)[2],
        &(*dst_mac)[3], &(*dst_mac)[4], &(*dst_mac)[5]) == 6) {
    } else {
        throw RuntimeException("Invalid MAC address format");
    }
    *this_mac = get_interface_mac_address(fd, interface);
    set_eth_packet_filter(fd, *dst_mac, !gateway_mode);
    return fd;
}


int main(int argc, char** argv) {
    std::string mac_address {"00:00:00:00:00:00"};
    std::string vcan_interface {"vcan0"};
    std::string interface {"lo"};
    bool gateway_mode = false;
    CLI::App app{"Utility converting ethernet l2 communication to vcan"};
    app.add_option("-v,--vcan", vcan_interface, "Use VCAN_INTERFACE for vcan")->type_name("VCAN_INTERFACE")->capture_default_str()->expected(1);
    app.add_option("-m,--mac", mac_address, "Use MAC address MAC_ADDRESS")->type_name("MAC_ADDRESS")->capture_default_str()->expected(1);
    app.add_option("-i,--interface", interface, "Use network interface INTERFACE")->type_name("INTERFACE")->capture_default_str()->expected(1);
    app.add_flag("-g,--gateway", gateway_mode, "Gateway mode for converting can to ethernet");
    CLI11_PARSE(app, argc, argv);

    int fd_vcan = open_vcan(vcan_interface);
    int fd_eth = open_eth(interface, mac_address, gateway_mode);

    pollfd poll_fds[2];
    poll_fds[0].fd = fd_vcan;
    poll_fds[0].events = POLLIN;
    poll_fds[1].fd = fd_eth;
    poll_fds[1].events = POLLIN;
    while(1) {
        int poll_result = poll(poll_fds, 2, 10);
        if (poll_result < 0) {
            throw RuntimeErrnoException("Poll error");
        } else if (poll_result > 0) {
            std::cout << "poll result " << poll_result << std::endl;
            if (poll_fds[0].revents) {
                canfd_frame can_frame;
                int nbytes = read(fd_vcan, &can_frame, sizeof(canfd_frame));
                if (nbytes <= 0) {
                    throw RuntimeErrnoException("vcan read error");
                }
                std::cout << "nbytes " << nbytes << std::endl;
                Payload payload {
                    .topic_id = htons(can_frame.can_id),
                    .length = can_frame.len,
                    .data = can_frame.data
                };
                std::memcpy(l2_frame_out.payload, &payload, 3);
                std::memcpy(l2_frame_out.payload+3, payload.data, payload.length);
                int result = send(fd_eth, &l2_frame_out, payload.length+3+22, 0);
                if (result < 0) {
                    throw RuntimeErrnoException("eth write error");
                }
            }
            if (poll_fds[1].revents) {
                std::cout << "eth" << std::endl;
                L2Frame frame {};
                int nbytes = read(fd_eth, &frame, sizeof(frame));
                if (nbytes <= 0) {
                    throw RuntimeErrnoException("eth read error");
                }
                std::cout << "eth nbytes " << nbytes << std::endl;
                uint8_t ptr = 0;
                while (ptr < sizeof(frame.payload)) {
                    Payload payload;
                    std::memcpy(&payload, &frame.payload[ptr], 3);
                    ptr += 3;
                    payload.topic_id = ntohs(payload.topic_id);
                    if (payload.topic_id == 0) {
                        break;
                    }
                    canfd_frame frame_out {
                        .can_id = payload.topic_id,
                        .len = payload.length,
                    };
                    std::memcpy(frame_out.data, &frame.payload[ptr], payload.length);
                    ptr += payload.length;
                    
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