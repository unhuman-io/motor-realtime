#include "motor_eth_l2.h"
#include <net/if.h>

namespace obot {

ssize_t EthL2RawFile::read(char *data, unsigned int length, bool write_read) {
    SocketFile::read(data, length, write_read);
    return 0;
}

ssize_t EthL2RawFile::write(const char *data, unsigned int length, bool write_read) {
    L2Frame frame;
    std::memcpy(frame.dst_mac, dst_mac_, 6);
    std::memcpy(frame.src_mac, src_mac_, 6);
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
    std::memcpy(frame.src_mac, src_mac_, 6);
    length = std::min(length, 64u);
    std::memcpy(frame.payload, data, length);
    frame.can_bus_id = can_bus_id_;
    frame.can_id = can_id_;
    frame.length = length+1;
    //frame.brs = 1;
    //frame.fdf = 1;
    frame.type = send_recv_frame_id_;
    SocketFile::write((const char *) &frame, sizeof(frame)-64+length, write_read);
    return 0;
}

ssize_t EthL2CANFile::writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
    SocketFile::writeread(data_out, length_out, data_in, length_in);
    return 0;
}

template<EthL2FileMode mode>
void MotorEthL2<mode>::get_interface_mac_address() {
    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFHWADDR, &ifr) == -1) {
        throw RuntimeException("ioctl SIOCGIFHWADDR failed for " + interface_ + ", error: " + std::to_string(errno) + ": " + strerror(errno));
    }
    std::memcpy(src_mac_, ifr.ifr_hwaddr.sa_data, 6);
}

template class MotorEthL2<EthL2FileMode::ETH_L2_RAW>;
template class MotorEthL2<EthL2FileMode::ETH_L2_CAN>;

} // namespace obot
