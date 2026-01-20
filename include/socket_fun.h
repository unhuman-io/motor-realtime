#pragma once

#include "socket_fun.h"


#include <string>
#include <cstdint>
#include <vector>
#include <array>


namespace obot {

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 1000;
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

struct TopicId {
    uint16_t node_id:4;
    uint16_t bus_id:3;
    uint16_t type:4;
};

struct Payload {
    uint16_t topic_id;
    uint16_t length;
    uint8_t* data;
};
constexpr int PAYLOAD_HEADER_SIZE = sizeof(Payload::topic_id) + sizeof(Payload::length);
static_assert(PAYLOAD_HEADER_SIZE == 4);

extern L2Frame l2_frame_out;

int open_vcan(std::string interface);
mac_t str2mac(std::string mac_str);
mac_t get_interface_mac_address(int fd, std::string interface);
void set_eth_packet_filter(int fd, mac_t mac, bool src = true);
int open_eth(std::string interface, std::string mac_address, bool gateway_mode, std::string gateway_mac);
std::vector<Payload> parse_eth_payload(uint8_t *frame_payload, ssize_t length);
}
