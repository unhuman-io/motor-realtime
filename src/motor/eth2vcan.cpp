#include "CLI11.hpp"
#include "exception.h"
#include "socket_fun.h"
#include <poll.h>
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

int main(int argc, char** argv) {
    std::string mac_address {"00:00:00:00:00:00"};
    std::string vcan_interface {"vcan0"};
    std::string interface {"lo"};
    std::string gateway_mac {"00:00:00:00:00:00"};
    CLI::App app{"Utility for converting ethernet l2 communication to vcan\n"
                 "    Example:\n"
                 "    sudo modprobe vcan\n"
                 "    sudo ip link add dev vcan0 type vcan\n"
                 "    sudo ip link set up vcan0\n"
                 "    eth2vcan -i eth0 -m 12:34:56:78:ab:cd\n"
                 "    motor_util -f vcan0"};
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
                std::memcpy(l2_frame_out.payload, &payload, PAYLOAD_HEADER_SIZE);
                std::memcpy(l2_frame_out.payload+PAYLOAD_HEADER_SIZE, payload.data, length);
                int result = send(fd_eth, &l2_frame_out, length+PAYLOAD_HEADER_SIZE+L2_HEADER_SIZE, 0);
                if (result < 0) {
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