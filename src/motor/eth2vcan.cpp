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
struct L2Frame {
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t reserved[8] = {};
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};
L2Frame l2_frame_out;

int open_eth(std::string interface, std::string mac_address) {
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

    if (sscanf(mac_address.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &l2_frame_out.dst_mac[0], &l2_frame_out.dst_mac[1], &l2_frame_out.dst_mac[2],
        &l2_frame_out.dst_mac[3], &l2_frame_out.dst_mac[4], &l2_frame_out.dst_mac[5]) == 6) {
    } else {
        throw RuntimeException("Invalid MAC address format");
    }
    return fd;
}


int main(int argc, char** argv) {
    std::string mac_address {"00:00:00:00:00:00"};
    std::string vcan_interface {"vcan0"};
    std::string interface {"lo"};
    CLI::App app{"Utility converting ethernet l2 communication to vcan"};
    app.add_option("-v,--vcan", vcan_interface, "Use VCAN_INTERFACE for vcan")->type_name("VCAN_INTERFACE")->capture_default_str()->expected(1);
    app.add_option("-m,--mac", mac_address, "Use MAC address MAC_ADDRESS")->type_name("MAC_ADDRESS")->capture_default_str()->expected(1);
    app.add_option("-i,--interface", interface, "Use network interface INTERFACE")->type_name("INTERFACE")->capture_default_str()->expected(1);
    CLI11_PARSE(app, argc, argv);

    int fd_vcan = open_vcan(vcan_interface);
    int fd_eth = open_eth(interface, mac_address);

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
                std::memcpy(l2_frame_out.payload, &can_frame, nbytes);
                int result = send(fd_eth, &l2_frame_out, nbytes+22, 0);
                if (result < 0) {
                    throw RuntimeErrnoException("eth write error");
                }
            }
            if (poll_fds[1].revents) {
                std::cout << "eth" << std::endl;
                L2Frame frame;
                int nbytes = read(fd_eth, &frame, sizeof(frame));
                if (nbytes <= 0) {
                    throw RuntimeErrnoException("eth read error");
                }
                std::cout << "eth nbytes " << nbytes << std::endl;
                int result = send(fd_vcan, &frame+22, nbytes-22, 0);
                if (result < 0) {
                    throw RuntimeErrnoException("vcan write error");
                }
            }
        }

    }

    return 0;
}