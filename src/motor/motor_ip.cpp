#include "motor_ip.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>
#include "poll.h"

namespace obot {

/// @brief Calculate the CRC16 of the given data.
///
/// Based on the CRC16-ANSI (MODBUS) specification.
/// Taken from: https://github.com/LacobusVentura/MODBUS-CRC16
/// @param data Pointer to a buffer.
/// @param size Size of the buffer.
/// @return CRC16 of the given data.
inline uint16_t crc16(const uint8_t* buf, uint32_t len) {  // NOLINT
  // NOLINTNEXTLINE(modernize-avoid-c-arrays)
  static const uint16_t kTable[256] = {
      0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780,
      0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1,
      0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801,
      0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40,
      0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680,
      0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0,
      0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
      0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
      0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
      0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1,
      0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200,
      0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141,
      0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480,
      0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0,
      0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01,
      0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
      0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381,
      0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
      0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01,
      0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40,
      0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81,
      0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1,
      0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100,
      0x81C1, 0x8081, 0x4040};

  uint8_t x_or = 0;
  uint16_t crc = 0xFFFF;  // NOLINT
  const int kBitsPerByte = 8;

  while (len--) {
    x_or = (*buf++) ^ crc;
    crc >>= kBitsPerByte;
    crc ^= kTable[x_or];
  }

  return crc;
}

void MotorIP::open() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    //std::cout << "socket fd " << fd_ << std::endl;
    addrinfo hints = {};
    hints.ai_family = AF_INET;      // IPv4
    hints.ai_socktype = SOCK_DGRAM; // UDP

    addrinfo *result, *res;
    int addr_info_result = getaddrinfo(ip_.c_str(), port_.c_str(), &hints, &result);
    if (addr_info_result != 0) {
      throw std::runtime_error("getaddrinfo failed for " + ip_ + ":" + port_ + ", error: " + std::to_string(addr_info_result));
    }

    char addrstr[100];
    void *ptr;
    res = result;
    int n_results = 0;
    while(res) {
        n_results++;
        socklen_t addr_len = 0;
        if (res->ai_family != AF_INET) {
          throw std::runtime_error(ip_ + ":" + port_ + ", not ipv4");
        }
        ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
        addr_len = sizeof(sockaddr_in);
        inet_ntop (res->ai_family, ptr, addrstr, 100);
        addrstr_ = addrstr;
        res = res->ai_next;
    }
    if (n_results != 1) {
      throw std::runtime_error(ip_ + ":" + port_ + ", n_results error: " + std::to_string(n_results));
    }
    std::memcpy(&addr_, result->ai_addr, result->ai_addrlen);

    getnameinfo((const sockaddr *) &addr_, sizeof(addr_), hostname_, sizeof(hostname_), NULL, 0, 0);

    freeaddrinfo(result);
    
    //flush();
}

int MotorIP::lock() {
    // lock file to prevent multiple instances from using the same port
    std::string lock_file = "/tmp/obot." + addrstr_ + ":" + port_ + ".lock";
    int fd_lock = ::open(lock_file.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_lock < 0) {
      throw std::runtime_error("Error opening lock file " + lock_file + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    int err = ::lseek(fd_lock, 0, SEEK_SET);
    if (err < 0) {
      throw std::runtime_error("Error lseek lock file " + lock_file + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    err = lockf(fd_lock, F_TLOCK, 0); 
    if (err) {
        std::cerr << "error locking " + lock_file;
        pid_t pid;
        int err2 = get_lock_pid(fd_lock, &pid);
        if (err2 == 0) {
            std::cerr << ", already locked by process: " << pid;
        }
        std::cerr << std::endl;
    }
    return err;
}

void UDPFile::flush() {
    int result;
    char c[1024];
    do {
      result = read(c, 1024);
    } while (result > 0);
}

int UDPFile::poll() {
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int poll_result = ::poll(&tmp, 1, timeout_ms_);
    if (poll_result == 0) {
        return ETIMEDOUT;
    } else if (poll_result < 0) {
        return poll_result;
    }
    return poll_result;
}

ssize_t UDPFile::_read(char * data, unsigned int length, bool write_read) {
  if (!write_read) {
    ObotPacket send_packet;
    send_packet.frame_id = recv_frame_id_;
    send_packet.length = 0;
    uint16_t crc = crc16((uint8_t*)&send_packet, 4);
    send_packet.data[send_packet.length] = (crc >> 8) & 0xFF;
    send_packet.data[send_packet.length+1] = crc & 0xFF;

    int send_result = sendto(fd_, &send_packet, send_packet.length+6, 0, (sockaddr *) &addr_, sizeof(addr_));
    if (send_result < 0) {
      return send_result;
    }
  }

  std::unique_lock<std::mutex> lk(rx_data_cv_m_);
  std::cv_status status = rx_data_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms_));

  if (status == std::cv_status::timeout) {
    //std::cout << "timed out" << std::endl;
    errno = ETIMEDOUT;
    return -1;
  } else {
    //std::cout << "cv result " << (int) status << std::endl;
    size_t len = std::min((size_t) length, rx_len_);
    std::memset(data, 0, length);
    std::memcpy(data, rx_buf_, len);
    return len;
  }
}

ssize_t UDPFile::read(char * data, unsigned int length, bool write_read) {
  ssize_t retval = _read(data, length, write_read);
  
  if (retval >= sizeof(APIControlPacket) && data[0] == 0) {
      // a control packet
      APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data);
      if (packet->type == TIMEOUT_REQUEST) {
          // timeout request
          if (retval == sizeof(APIControlPacket)) {
              // timeout request
              // retriggers the read with the new timeout
              uint32_t old_timeout_ms = timeout_ms_;
              timeout_ms_ += packet->timeout_request.timeout_us/1000;
              ssize_t retval = _read(data, length, write_read);
              timeout_ms_ = old_timeout_ms;
              return retval;
          }
      } else if (packet->type == LONG_PACKET) {
          // long packet
          uint16_t total_length = packet->long_packet.total_length;
          uint16_t packet_number = packet->long_packet.packet_number;
          const uint8_t header_size = sizeof(APIControlPacket);
          uint16_t total_count_received = retval - header_size;
          if (total_length > length) {
              // too long
              return -EINVAL;
          }
          std::memcpy(data, data + header_size, total_count_received);
          while (total_length > total_count_received) {
              // assemble multiple packets
              char * data_ptr = data + total_count_received;
              retval = _read(data_ptr, length, write_read);
              if (retval < 0) {
                  return retval;
              }
              std::memcpy(data_ptr, data_ptr + header_size, retval - header_size);
              total_count_received += retval - header_size;
              // ignoring packet_number
          }
          return total_count_received;
      }
  }
  return retval;
}

ssize_t UDPFile::write(const char * data, unsigned int length, bool write_read) {
    ObotPacket packet;
    // Pack data into packet
    // todo: use generate_packet()
    std::memcpy(packet.data, data, length);
    if (write_read) {
      packet.frame_id = send_recv_frame_id_;
    } else {
      packet.frame_id = send_frame_id_;
    }
    
    packet.length = length;
    //std::cout << "send length " << length << std::endl;
    // Calculate CRC of command payload
    uint16_t crc = crc16((uint8_t*)&packet, length+4);
    packet.data[length] = (crc >> 8) & 0xFF;
    packet.data[length+1] = crc & 0xFF;

    std::memcpy(packet.data, data, length);
    int send_result = sendto(fd_, &packet, 6+length, 0, (sockaddr *) &addr_, sizeof(addr_));
    return send_result;
}

ssize_t UDPFile::writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
    for (int i = 0; i<3; i++) {
      int write_result = write(data_out, length_out, true);
      if (write_result < 0) {
        //std::cout << "write result " << write_result << std::endl;
        return write_result;
      }
      int read_result = read(data_in, length_in, true);
      if (read_result <= 0) {
        // retry
        continue;
      }
      //std::cout << "api " << read_result << " " << data_in[0] << std::endl;
      return read_result;
    }
    return -1;
}

void UDPFile::rx_callback(const uint8_t* buf, uint16_t len) {
  {
    std::lock_guard<std::mutex> lk(rx_data_cv_m_);
    std::memcpy(rx_buf_, buf, len);
    rx_len_ = len;
  }
  rx_data_cv_.notify_one();
}

MotorIP::~MotorIP() {
  terminate_ = true;
  rx_thread_.join(); // todo add timeout
}

bool MotorIP::connect() {
    fd_flags_ = fcntl(fd_, F_GETFL);
    messages_version_ = operator[]("messages_version").get();
    if (messages_version_ == "") {
      return false;
    }
    name_ = operator[]("name").get();
    if (name_ == "") {
      name_ = ip() + ":" + std::to_string(port());
    }
    version_ = operator[]("version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
    serial_number_ = operator[]("serial").get();
    dev_path_ = hostname_;
    base_path_ = addrstr_;
    devnum_ = port();
    return true;  
}

void MotorIP::set_timeout_ms(int timeout_ms) {
    static_cast<UDPFile*>(motor_txt_.get())->timeout_ms_ = timeout_ms;
    realtime_communication_.timeout_ms_ = timeout_ms;
}

ssize_t MotorIP::read() {
  //std::cout << "read " << std::endl;
  int ret = realtime_communication_.read((char *) &status_, sizeof(status_));
  if (ret < 0) {
    return 0;
  }
  return ret;
}

ssize_t MotorIP::write() {
  //std::cout << "write " << std::endl;
  return realtime_communication_.write((char *) &command_, sizeof(command_));
}

void MotorIP::rx_data() {
  //std::cout << "rx_data started, fd_ " << fd_ << std::endl;
  while(1) {
    // assume blocking i/o
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int poll_result = ::poll(&tmp, 1, 5 /* ms */);
    if (poll_result > 0) {
      int result = recv(fd_, rx_lin_buffer_, RX_BUFFER_SIZE, 0);
      //std::cout << "read result " << result << ", read idx " << current_read_idx_ << std::endl;
      if (result < 0) {
        throw std::runtime_error("Error rx_data: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
      }
      for (int i=0; i<result; i++) {
        rx_buffer_[current_read_idx_] = rx_lin_buffer_[i];
        current_read_idx_ = (current_read_idx_ + 1) % RX_BUFFER_SIZE;
      }
      parser_.process((current_read_idx_ - 1) % RX_BUFFER_SIZE);
    }
    if (terminate_) {
      return;
    }
  }
}

}; // namespace obot
