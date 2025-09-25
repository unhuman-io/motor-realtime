#include "motor_eth_l2.h"

namespace obot {

ssize_t EthL2File::read(char *data, unsigned int length, bool write_read) {
    SocketFile::read(data, length, write_read);
    return 0;
}

ssize_t EthL2File::write(const char *data, unsigned int length, bool write_read) {
    L2Frame frame;
    std::memcpy(frame.dst_mac, dst_mac_, 6);
    std::memcpy(frame.payload, data, length);
    SocketFile::write((const char *) &frame, length+14, write_read);
    return 0;
}

ssize_t EthL2File::writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
    SocketFile::writeread(data_out, length_out, data_in, length_in);
    return 0;
}

} // namespace obot
