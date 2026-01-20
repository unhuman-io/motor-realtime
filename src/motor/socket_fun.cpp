#include "socket_fun.h"
#include "exception.h"

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
#include <iostream>

namespace obot {

L2Frame l2_frame_out;

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

void set_eth_packet_filter(int fd, mac_t mac, bool src) {
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
        if (payload.length > 0) {
            payload.data = &frame_payload[ptr];
        }
        ptr += payload.length;
        payloads.push_back(payload);
        std::cout << "\ttopic_id: " << std::hex << payload.topic_id << std::dec << ", length: " << (int) payload.length << std::endl;
    }
    std::cout << "\t" << payloads.size() << " payloads" << std::endl;
    if (ptr > length) {
        throw RuntimeException("payload sum error, " + std::to_string(payloads.size()) + " messages, total length " + std::to_string(ptr));
    }
    return payloads;
}
} // namespace obot
