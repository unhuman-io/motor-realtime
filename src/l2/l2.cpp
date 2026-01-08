
#include "l2.h"
#include "exception.h"
#include <netdb.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <cstring>
#include <poll.h>
#include <linux/filter.h>
#include <iostream>

namespace obot {

mac_address_t mac_ascii_to_raw(std::string mac_ascii) {
    mac_address_t mac;
    if (sscanf(mac_ascii.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &mac[0], &mac[1], &mac[2],
        &mac[3], &mac[4], &mac[5]) == 6) {
    } else {
        throw RuntimeException("Invalid MAC address format: " + mac_ascii);
    }
    return mac;
}

void L2Socket::open() {
  fd_ = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
  if (fd_ < 0) {
    throw RuntimeErrnoException("socket create failed");
  }

  sockaddr_ll server_addr = {};
  server_addr.sll_family = AF_PACKET;
  server_addr.sll_protocol = htons(0x88B5);
  server_addr.sll_ifindex = if_nametoindex(interface_.c_str());

  int retval = bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (retval < 0) {
    throw RuntimeErrnoException("bind failed");
  }
}

void L2Socket::send(const char * data, std::size_t length) {
    std::memcpy(&frame_out_.payload, data, length);
    int send_result = ::send(fd_, &frame_out_, length+14, 0);
}

int L2Socket::recv(char * data, std::size_t length, int timeout_us) {
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int result = ::poll(&tmp, 1, 0);
    if (result > 0) {
      result = ::recv(fd_, &frame_in_, sizeof(frame_in_), 0);
      result -= 24;
      if (result < 0) {
        throw RuntimeErrnoException("Error recv");
      }
      length = std::min(length, static_cast<std::size_t>(result));
      std::memcpy(data, frame_in_.payload, length);
    } else {
      length = 0;
    }
    return length;
}

std::vector<Packet> L2Socket::parse_payload(uint8_t *payload, std::size_t length) {
  int ptr = 0;
  std::vector<Packet> packets;
  while(ptr <= length + 3) {
    int packet_length = payload[ptr+2];
    //std::cout << "packet " << packet_length << std::endl;
    if (ptr + 3 + packet_length <= length) {
      packets.push_back({});
      std::memcpy(&packets[packets.size()-1], &payload[ptr], packet_length+3);
      ptr += 3 + packet_length;
    } else {
      if (packet_length != 0) {
        throw RuntimeException("invalid packet length");
      }
      break;
    }
  }
  return packets;
}

L2Device::L2Device(std::string interface, std::string mac_address) : L2Socket(interface) {
  mac_ = mac_ascii_to_raw(mac_address);
  std::memcpy(frame_out_.src_mac, &mac_, 6);
  set_packet_filter();
}

void L2Device::set_packet_filter() {
    uint32_t dest_word1;
    std::memcpy(&dest_word1, frame_out_.src_mac, 4);
    dest_word1 = htonl(dest_word1);
    uint16_t dest_word2;
    std::memcpy(&dest_word2, frame_out_.src_mac+4, 2);
    dest_word2 = htons(dest_word2);
    // Set Berkeley Packet Filter to only receive packets with src_mac == dst_mac_
    struct sock_filter bpf_code[] = {
        // Load first 4 bytes of Ethernet dst MAC (offset 6)
        { BPF_LD+BPF_W+BPF_ABS, 0, 0, 0x00000000 }, // BPF_LD+BPF_W+BPF_ABS = 0x20, offset 0
        // Compare with dst_mac_[0..3]
        { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, dest_word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15

        // Load next 2 bytes of Ethernet dst MAC (offset 10)
        { BPF_LD+BPF_H+BPF_ABS, 0, 0, 0x00000004 }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 4
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

} // namespace obot
