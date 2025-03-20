#include "motor_uart_obot.h"
#include <errno.h>
#include "poll.h"
#include <asm/termbits.h>
#include <condition_variable>

#include <thread>
#include <chrono>

namespace obot {

class UartObotTextFile : public TextFile {
  public:
      UartObotTextFile(figure::ProtocolParser &parser) :
          parser_(parser) {}
      void register_callbacks();
      virtual ssize_t read(char * data, unsigned int length) override;
      virtual ssize_t write(const char * data, unsigned int length) override;
      virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) override;
      void set_timeout_ms(int timeout_ms) { timeout_ms_ = timeout_ms; }
      uint8_t send_frame_id_ = 4; // text command
      uint8_t recv_frame_id_ = 5; // text response
      uint8_t send_recv_frame_id_ = 4; // both
      int fd_;
  private:
      figure::ProtocolParser &parser_;
      std::condition_variable rx_data_cv_;
      std::mutex rx_data_cv_m_; // protects rx_data_cv_, rx_buf_ and rx_len_
      uint8_t rx_buf_[1024];
      size_t rx_len_ = 0;
      uint32_t timeout_ms_ = 100;
  };

MotorUARTObot::MotorUARTObot(std::string dev_path, uint32_t baud_rate) {
  dev_path_ = dev_path;
  int result = open();
  if (result < 0) {
    throw std::runtime_error("Error opening " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  motor_txt_ = std::move(std::unique_ptr<UartObotTextFile>(new UartObotTextFile(parser_)));
  UartObotTextFile * text_mailbox = static_cast<UartObotTextFile *>(motor_txt_.get());
  text_mailbox->fd_ = fd_;
  text_mailbox->register_callbacks();

  parser_.registerCallback(2, [this](const uint8_t* buf, uint16_t len) // status
  {
    {
      std::lock_guard<std::mutex> lk(rx_data_cv_m_);
      std::memcpy(rx_buf_, buf, len);
      rx_len_ = len;
    }
    rx_data_cv_.notify_one();
  });

  // only one item can access uart devices due to protocol
  result = lock();
  if (result < 0) {
    throw std::runtime_error("Error locking: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  set_baud_rate(baud_rate);

  rx_thread_ = std::thread([this]{ this->rx_data(); });
  
  version_ = operator[]("version").get();
  messages_version_ = operator[]("messages_version").get();
  name_ = operator[]("name").get();
  board_name_ = operator[]("board_name").get();
  board_rev_ = operator[]("board_rev").get();
  board_num_ = operator[]("board_num").get();
  config_ = operator[]("config").get();
  serial_number_ = operator[]("serial").get();
  
}

void MotorUARTObot::set_timeout_ms(int timeout_ms) {
  timeout_ms_ = timeout_ms;
  UartObotTextFile * text_mailbox = static_cast<UartObotTextFile *>(motor_txt_.get());
  text_mailbox->set_timeout_ms(timeout_ms);
}

void MotorUARTObot::set_baud_rate(uint32_t baud_rate) {
  int result;
  struct termios2 tio2 = {};

  result = ioctl(fd_, TCGETS2, &tio2);
  if (result < 0) {
    throw std::runtime_error("Error tcgets2: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  tio2.c_cflag = CS8 | CREAD | CLOCAL | CBAUDEX;
  tio2.c_lflag = 0;
  tio2.c_iflag = 0;
  tio2.c_oflag = 0;
  tio2.c_cc[VMIN] = 0;
  tio2.c_cc[VTIME] = 1;
  tio2.c_ispeed = baud_rate;
  tio2.c_ospeed = baud_rate;
  result = ioctl(fd_, TCSETS2, &tio2);

  if (result < 0) {
    throw std::runtime_error("Error tcsets2: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }

  result = ioctl(fd_, TCGETS2, &tio2);
  if (result < 0) {
    throw std::runtime_error("Error tcgets2: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  if (tio2.c_ispeed != baud_rate || tio2.c_ospeed != baud_rate) {
    throw std::runtime_error("Error setting baud rate " + std::to_string(baud_rate) + " on " + dev_path_);
  }
}

ssize_t MotorUARTObot::read() {
  uint8_t packet_size;
  uint8_t * packet_out = parser_.generatePacket(nullptr, 0, 2, &packet_size); // get status
  int retval = ::write(fd_, packet_out, packet_size);
  if (retval != packet_size) {
    std::cerr << "write error " << retval << std::endl;
    return retval;
  }

  std::unique_lock<std::mutex> lk(rx_data_cv_m_);
  bool status = rx_data_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms_), [this]{ return rx_len_ != 0; });
  if (status == false) {
    errno = ETIMEDOUT;
    return -1;
  } else {
    size_t len = std::min(sizeof(status_), rx_len_);
    std::memset(&status_, 0, sizeof(status_));
    std::memcpy(&status_, rx_buf_, len);
    rx_len_ = 0;
    return len;
  }
}

ssize_t MotorUARTObot::write() {
  uint8_t packet_size;
  uint8_t * packet_out = parser_.generatePacket((uint8_t *) &command_, sizeof(command_), 1, &packet_size);
  int retval = ::write(fd_, packet_out, packet_size);
  if (retval != packet_size) {
    std::cerr << "write error " << retval << std::endl;
  }
  return retval;
}

void MotorUARTObot::rx_data() {
  while(1) {
    // assume blocking i/o
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int poll_result = ::poll(&tmp, 1, 5 /* ms */);
    if (poll_result > 0) {
      int result = ::read(fd_, rx_lin_buffer_, RX_BUFFER_SIZE);
      // std::cout << "read result " << result << ", read idx " << current_read_idx_ << std::endl;
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

MotorUARTObot::~MotorUARTObot() {
  terminate_ = true;
  rx_thread_.join(); // todo add timeout
}

void UartObotTextFile::register_callbacks() {
  parser_.registerCallback(recv_frame_id_, [this](const uint8_t* buf, uint16_t len)
  {
    {
      std::lock_guard<std::mutex> lk(rx_data_cv_m_);
      std::memcpy(rx_buf_, buf, len);
      rx_len_ = len;
    }
    rx_data_cv_.notify_one();
  });
}

ssize_t UartObotTextFile::read(char * data, unsigned int length) {
  std::unique_lock<std::mutex> lk(rx_data_cv_m_);
  bool status = rx_data_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms_), [this]{ return rx_len_ != 0; });
  if (status == false) {
    errno = ETIMEDOUT;
    return -1;
  } else {
    size_t len = std::min((size_t) length, rx_len_);
    std::memset(data, 0, length);
    std::memcpy(data, rx_buf_, len);
    rx_len_ = 0;
    return len;
  }
}


ssize_t UartObotTextFile::write(const char * data, unsigned int length) {
  uint8_t packet_size;
  uint8_t * packet_out = parser_.generatePacket((uint8_t *) data, length, send_recv_frame_id_, &packet_size);
  int retval = ::write(fd_, packet_out, packet_size);
  if (retval != packet_size) {
    std::cerr << "write error " << retval << std::endl;
  }
  return length;
}

ssize_t UartObotTextFile::writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
  int retval = write(data_out, length_out);
  if (retval < 0) {
    std::cerr << "writeread write error " << retval << std::endl;
    return retval;
  }
  retval = read(data_in, length_in);
  if (retval < 0) {
    std::cerr << "writeread read error " << retval << std::endl;
    return retval;
  }
  return retval;
}


}; // namespace obot
