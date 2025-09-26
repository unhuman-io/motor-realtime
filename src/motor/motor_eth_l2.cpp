#include "motor_eth_l2.h"

namespace obot {

ssize_t EthL2RawFile::read(char *data, unsigned int length, bool write_read) {
    SocketFile::read(data, length, write_read);
    return 0;
}

ssize_t EthL2RawFile::write(const char *data, unsigned int length, bool write_read) {
    L2Frame frame;
    std::memcpy(frame.dst_mac, dst_mac_, 6);
    std::memcpy(frame.payload, data, length);
    SocketFile::write((const char *) &frame, length+14, write_read);
    return 0;
}

ssize_t EthL2RawFile::writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
    SocketFile::writeread(data_out, length_out, data_in, length_in);
    return 0;
}

ssize_t EthL2CANFile::read(char *data, unsigned int length, bool write_read) {
    SocketFile::read(data, length, write_read);
    return 0;
}

ssize_t EthL2CANFile::write(const char *data, unsigned int length, bool write_read) {
    L2CANFrame frame;
    std::memcpy(frame.dst_mac, dst_mac_, 6);
    length = std::min(length, 64u);
    std::memcpy(frame.payload, data, length);
    frame.can_bus_id = can_bus_id_;
    frame.can_id = can_id_;
    frame.length = length;
    frame.brs = 1;
    frame.fdf = 1;
    SocketFile::write((const char *) &frame, sizeof(frame)-64+length, write_read);
    return 0;
}

ssize_t EthL2CANFile::writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
    SocketFile::writeread(data_out, length_out, data_in, length_in);
    return 0;
}

} // namespace obot
