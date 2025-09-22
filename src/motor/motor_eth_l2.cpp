#include "motor_eth_l2.h"

namespace obot {

ssize_t EthL2File::read(char *data, unsigned int length, bool write_read) {
    SocketFile::read(data, length, write_read);
    return 0;
}

ssize_t EthL2File::write(const char *data, unsigned int length, bool write_read) {
    char buffer[length + 14];
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[2] = 0xFF;
    buffer[3] = 0xFF;
    buffer[4] = 0xFF;
    buffer[5] = 0xFF;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    buffer[8] = 0x00;
    buffer[9] = 0x00;
    buffer[10] = 0x00;
    buffer[11] = 0x00;
    buffer[12] = 0x88;
    buffer[13] = 0xB5; // Ethertype
    std::memcpy(buffer + 14, data, length);
    SocketFile::write(buffer, sizeof(buffer), write_read);
    return 0;
}

ssize_t EthL2File::writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
    std::cout << "writeread length_out " << length_out << ", length_in " << length_in << std::endl;
    // char buffer[length_out + 14];
    // buffer[0] = 0xFF;
    // buffer[1] = 0xFF;
    // buffer[2] = 0xFF;
    // buffer[3] = 0xFF;
    // buffer[4] = 0xFF;
    // buffer[5] = 0xFF;
    // buffer[6] = 0x00;
    // buffer[7] = 0x00;
    // buffer[8] = 0x00;
    // buffer[9] = 0x00;
    // buffer[10] = 0x00;
    // buffer[11] = 0x00;
    // buffer[12] = 0x88;
    // buffer[13] = 0xB5; // Ethertype
    // std::memcpy(buffer + 14, data_out, length_out);
    SocketFile::writeread(data_out, length_out, data_in, length_in);
    return 0;
}

} // namespace obot
