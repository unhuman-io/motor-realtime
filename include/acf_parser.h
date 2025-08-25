#pragma once

#include <cstdint>
#include <functional>

namespace obot {


struct ACFPacket {
    uint32_t header[6];
    //struct ACFMessage {
        uint8_t acf_message_type;
        uint8_t length;
        uint8_t pad:2;
        uint8_t mtv:1;
        uint8_t rtr:1;
        uint8_t eff:1;
        uint8_t brs:1;
        uint8_t fdf:1;
        uint8_t esi:1;
        uint8_t reserved:3;
        uint8_t can_bus_id:5;
        uint32_t timestamp;
        uint32_t can_id:7;
        uint32_t frame_id:4;
        uint32_t reserved_id:21;
        uint8_t data[64];
   // };
};

class ACFParser {
public:
  using callback_t = std::function<void(const uint8_t*, uint16_t)>;
    ACFParser(uint8_t* buffer, size_t buffer_size) : buffer_(buffer), buffer_size_(buffer_size) {}
    void process(const uint32_t latest_idx) {
        uint8_t *ptr = (uint8_t*) &packet_in_;
        for(int i=last_idx_; i<latest_idx; i++) {
            *ptr++ = buffer_[i];
        }
        last_idx_ = latest_idx;
        if (packet_in_.can_id == can_id_) {
            callbacks[packet_in_.frame_id](packet_in_.data, packet_in_.length);
        }
    }

    void registerCallback(uint8_t frame_id, callback_t&& callback) {
        callbacks[frame_id] = std::move(callback);
    }
    uint8_t* generatePacket(const uint8_t* buffer, const uint32_t buffer_size, const uint8_t frame_id,
                            uint8_t* gen_packet_size) {
        std::memset(&packet_buffer_, 0, sizeof(packet_buffer_));
        packet_buffer_.length = 64;
        packet_buffer_.fdf = 1;
        packet_buffer_.brs = 1;
        packet_buffer_.can_bus_id = can_bus_id_;
        packet_buffer_.frame_id = frame_id;
        packet_buffer_.can_id = can_id_;
        *gen_packet_size = sizeof(packet_buffer_);
        std::memcpy(packet_buffer_.data, buffer, std::min((size_t)64, (size_t)buffer_size));
        return (uint8_t *) &packet_buffer_;
    }

private:
    uint8_t* buffer_;
    ACFPacket packet_buffer_;
    ACFPacket packet_in_;
    size_t buffer_size_;
    uint32_t last_idx_ = 0;
    uint8_t can_bus_id_ = 0;
    uint8_t can_id_ = 1;
    callback_t callbacks[16];
};

}; // namespace obot
