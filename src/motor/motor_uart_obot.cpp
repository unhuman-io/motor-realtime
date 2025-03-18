#include "motor_uart_obot.h"
#include <errno.h>
#include "poll.h"
#include <asm/termbits.h>

#include <thread>
#include <chrono>

namespace obot {

class UartObotTextFile : public TextFile {
  public:
      UartObotTextFile(figure::ProtocolParser &parser) :
          parser_(parser) {
      }
      virtual ssize_t read(char * data, unsigned int length) override;
      virtual ssize_t write(const char * data, unsigned int length) override;
      virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) override;
      uint8_t send_frame_id_ = 4; // text command
      uint8_t recv_frame_id_ = 5; // text status
      uint8_t send_recv_frame_id_ = 4; // both
      int fd_;
  private:
      figure::ProtocolParser parser_;
  };

MotorUARTObot::MotorUARTObot(std::string dev_path, uint32_t baud_rate) {
  dev_path_ = dev_path;
  int result = open();
  if (result < 0) {
    throw std::runtime_error("Error opening " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  realtime_mailbox_.fd_ = fd_;
  motor_txt_ = std::move(std::unique_ptr<UartObotTextFile>(new UartObotTextFile(parser_)));
  UartObotTextFile * text_mailbox = static_cast<UartObotTextFile *>(motor_txt_.get());
  text_mailbox->fd_ = fd_;

  // only one item can access uart devices due to protocol
  result = lock();
  if (result < 0) {
    throw std::runtime_error("Error locking: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  set_baud_rate(baud_rate);
  // if (sync() < 0) {
  //   throw std::runtime_error("Error syncing: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  // }
  
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
}

void MotorUARTObot::set_baud_rate(uint32_t baud_rate) {
  int result;
  struct termios2 tio2 = {};

  ioctl(fd_, TCGETS2, &tio2);
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
}

ssize_t MotorUARTObot::read() {
  //static int count = 0;
  ssize_t result = ::read(fd_, &status_, sizeof(status_));
  if (status_.host_timestamp_received != command_.host_timestamp) {
    // count++;
    // if (count > 1){
    // std::cout << "host timestamp received: " << status_.host_timestamp_received << ", sent" << command_.host_timestamp << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(8000));
    // std::cout << "result " << result << std::endl;
    // }
    
  }
  return result;
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

ssize_t UartObotTextFile::read(char * data, unsigned int length) {
  int retval;
  retval = ::read(fd_, data, length);
  // std::cout << "read " << retval << std::endl;
  if (retval < 0) {
    //read_error_++;
    std::cerr << "read error " << retval << std::endl;
  } else {
    std::cout << "read " << retval << std::endl;
  }
  return retval;
}


ssize_t UartObotTextFile::write(const char * data, unsigned int length) {
  uint8_t packet_size;
  uint8_t * packet_out = parser_.generatePacket((uint8_t *) data, length, send_recv_frame_id_, &packet_size);
  int retval = ::write(fd_, packet_out, packet_size);
  if (retval != packet_size) {
    std::cerr << "write error " << (char *) (retval + 4) << std::endl;
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
