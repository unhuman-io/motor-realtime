#pragma once
#include <string>
#include <cstdint>
#include <array>
#include "exception.h"
#include <vector>

namespace obot {

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 1000;
struct L2Frame {
    uint8_t dst_mac[6] = {};
    uint8_t src_mac[6] = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t reserved[8] = {};
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};

struct Packet {
    struct {
      uint16_t node_id:7;
      uint16_t type:4;
    };
    uint8_t length;
    uint8_t data[255];
};


using mac_address_t = std::array<uint8_t, 6>;
mac_address_t mac_ascii_to_raw(std::string mac_ascii);

class L2Socket {
  public:
    L2Socket(std::string interface) : interface_(interface) { open(); }
    void open();
    void send(const char *data, std::size_t length);
    int recv(char * data, std::size_t length, int timeout_us = 0);
    std::vector<Packet> parse_payload(uint8_t *payload, std::size_t length);
  protected:
    int fd_;
    mac_address_t mac_;
    L2Frame frame_out_;
    L2Frame frame_in_;
  private:
    std::string interface_;
};

class L2Device : public L2Socket {
  public:
    L2Device(std::string interface, std::string mac_address);
  private:
    void set_packet_filter();
};

} // namespace obot