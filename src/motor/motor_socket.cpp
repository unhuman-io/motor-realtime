#include "motor_socket.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <errno.h>
#include "poll.h"

namespace obot {

void MotorSocket::open() {
    fd_ = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd_ < 0) {
      throw RuntimeException("socket failed for " + address_ + ", error: " + std::to_string(errno) + ": " + strerror(errno));
    }

    sockaddr_ll server_addr = {};
    server_addr.sll_family = AF_PACKET;
    server_addr.sll_protocol = htons(0x88B5);
    server_addr.sll_ifindex = if_nametoindex(interface_.c_str());
  
    int retval = bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (retval < 0) {
      throw RuntimeException("bind failed for " + address_ + ", error: " + std::to_string(errno) + ": " + strerror(errno));
    }
    
    create_communication_lock();
    //flush();
}

int MotorSocket::create_communication_lock() {
    // lock file to prevent multiple instances from using the same port
    std::string lock_file = "/tmp/obot." + address_ + ".lock";
    fd_communication_lock_ = ::open(lock_file.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_communication_lock_ < 0) {
      throw RuntimeException("Error opening lock file " + lock_file + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    int err = ::lseek(fd_communication_lock_, 0, SEEK_SET);
    if (err < 0) {
      throw RuntimeException("Error lseek lock file " + lock_file + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    return err;
}

int SocketFile::lock_communication() {
    communication_lock_count_++;
    if (communication_lock_count_ > 1) {
        return 0; // already locked
    }
    int err = lockf(fd_communication_lock_, F_LOCK, 0); 
    if (err) {
        std::cerr << "error locking " + std::to_string(errno) + ": " + strerror(errno);
        pid_t pid;
        int err2 = get_lock_pid(fd_communication_lock_, &pid);
        if (err2 == 0) {
            std::cerr << ", already locked by process: " << pid;
        }
        std::cerr << std::endl;
    }
    return err;
}

int SocketFile::unlock_communication() {
    communication_lock_count_--;
    if (communication_lock_count_ > 0) {
        return 0; // not ready to unlock yet
    }
    int err = lockf(fd_communication_lock_, F_ULOCK, 0);
    if (err) {
        std::cerr << "error unlocking " + std::to_string(errno) + ": " + strerror(errno);
        pid_t pid;
        int err2 = get_lock_pid(fd_communication_lock_, &pid);
        if (err2 == 0) {
            std::cerr << ", locked by process: " << pid;
        }
        std::cerr << std::endl;
    }
    return err;
}

void SocketFile::flush() {
    int result;
    char c[1024];
    do {
      result = read(c, 1024);
    } while (result > 0);
}

int SocketFile::poll() {
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

ssize_t SocketFile::_read(char * data, unsigned int length, bool write_read) {
  if (!write_read) {
    char buffer[length + 14];
    std::memcpy(buffer + 14, data, length);
    int send_result = send(fd_, buffer, sizeof(buffer), 0);
    if (send_result < 0) {
      return send_result;
    }
  }

  {
    std::lock_guard<std::mutex> lk(rx_data_request_cv_m_);
    rx_data_request_ = true;
  }
  rx_data_request_cv_.notify_one();
  std::unique_lock<std::mutex> lk(rx_data_cv_m_);
  bool status = rx_data_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms_), [this]{ return rx_received_ != false; });

  if (status == false) {
    errno = ETIMEDOUT;
    return -1;
  } else {
    size_t len = std::min((size_t) length, rx_len_);
    std::memset(data, 0, length);
    std::memcpy(data, rx_buf_, len);
    rx_received_ = false;
    return len;
  }
}

ssize_t SocketFile::read(char * data, unsigned int length, bool write_read) {
  lock_communication();
  ssize_t retval = _read(data, length, write_read);
  
  if (api_mode_) {
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
              unlock_communication();
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
              unlock_communication();
              return -EINVAL;
          }
          std::memmove(data, data + header_size, total_count_received);
          while (total_length > total_count_received) {
              // assemble multiple packets
              char * data_ptr = data + total_count_received;
              char buf[length];
              retval = _read(buf, length, write_read);
              if (retval < 0) {
                  unlock_communication();
                  return retval;
              }
              APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(buf);
              if (packet->type != LONG_PACKET) {
                  std::cerr << "Error: expected long packet, got " << packet->type << std::endl;
                  unlock_communication();
                  return -EINVAL;
              }
              if (packet->long_packet.packet_number != ++packet_number) {
                  std::cerr << "Error: expected packet number " << packet_number << ", got " << packet->long_packet.packet_number << std::endl;
                  unlock_communication();
                  return -EINVAL;
              }
              total_count_received += retval - header_size;
              std::memmove(data_ptr, buf + header_size, retval - header_size);
              // ignoring packet_number
          }
          if (total_count_received != total_length) {
            std::cerr << "Error: expected " << total_length << " bytes, got " << total_count_received << std::endl;
            unlock_communication();
            return -EINVAL;
          }
          retval = total_count_received;
        }
    }
  }
  unlock_communication();
  return retval;
}

ssize_t SocketFile::write(const char * data, unsigned int length, bool write_read) {
    //std::cout << "write length " << length << ", " << data << std::endl;
    lock_communication();
    char buffer[length];
    std::memcpy(buffer, data, length);
    int send_result = send(fd_, buffer, sizeof(buffer), 0);
    unlock_communication();
    return send_result;
}

ssize_t SocketFile::writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
    lock_communication();
    int retval = write(data_out, length_out, true);
    if (retval < 0) {
      unlock_communication();
      return retval;
    }
    retval = read(data_in, length_in, true);

    unlock_communication();
    return retval;
}

void SocketFile::rx_callback(const uint8_t* buf, uint16_t len) {
  std::unique_lock<std::mutex> lk(rx_data_request_cv_m_);
  bool status = rx_data_request_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms_), [this]{ return rx_data_request_; });
  if (status == false) {
    throw RuntimeException("rx_callback timeout - data received without active request");
  }
  rx_data_request_ = false;
  {
    std::lock_guard<std::mutex> lk(rx_data_cv_m_);
    std::memcpy(rx_buf_, buf, len);
    rx_len_ = len;
    rx_received_ = true;
  }
  rx_data_cv_.notify_one();
}

MotorSocket::~MotorSocket() {
  terminate_ = true;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

bool MotorSocket::connect() {
    fd_flags_ = fcntl(fd_, F_GETFL);
    messages_version_ = operator[]("messages_version").get();
    if (messages_version_ == "") {
      return false;
    }
    name_ = operator[]("name").get();
    if (name_ == "") {
      name_ = address_;
    }
    version_ = operator[]("version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
    serial_number_ = operator[]("serial").get();
    dev_path_ = interface_;
    base_path_ = address_;
    devnum_ = 0;
    return true;  
}

void MotorSocket::set_timeout_ms(int timeout_ms) {
    static_cast<SocketFile*>(motor_txt_.get())->timeout_ms_ = timeout_ms;
    realtime_communication_.timeout_ms_ = timeout_ms;
}

ssize_t MotorSocket::read() {
  //std::cout << "read " << std::endl;
  int ret = realtime_communication_.read((char *) &status_, sizeof(status_));
  if (ret < 0) {
    std::cout << "read error " << ret << std::endl;
  }
  return ret;
}

ssize_t MotorSocket::write() {
  // std::cout << "write " << std::endl;
  return realtime_communication_.write((char *) &command_, sizeof(command_));
}

void MotorSocket::rx_data() {
  //std::cout << "rx_data started, fd_ " << fd_ << std::endl;
  try {
    while(1) {
      // assume blocking i/o
      pollfd tmp;
      tmp.fd = fd_;
      tmp.events = POLLIN;
      int poll_result = ::poll(&tmp, 1, 5 /* ms */);
      if (poll_result > 0) {
        int result = recv(fd_, rx_lin_buffer_, RX_BUFFER_SIZE, 0);
        // std::cout << "read result " << result << ", read idx " << current_read_idx_ << std::endl;
        if (result < 0) {
          throw RuntimeException("Error rx_data: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
        }
        for (int i=0; i<result; i++) {
          rx_buffer_[current_read_idx_] = rx_lin_buffer_[i];
          current_read_idx_ = (current_read_idx_ + 1) % RX_BUFFER_SIZE;
        }
        dynamic_cast<SocketFile*>(motor_txt_.get())->rx_callback(rx_lin_buffer_, result);
        //parser_.process((current_read_idx_ - 1) % RX_BUFFER_SIZE);
      }
      if (terminate_) {
        return;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "rx_thread caught exception: " << e.what() << std::endl;
    std::cerr << "rx_thread terminating" << std::endl;
  }
}

}; // namespace obot
