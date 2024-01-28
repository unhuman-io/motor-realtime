#include "motor_uart.h"
#include <errno.h>
#include "poll.h"
#include <termios.h>

#include <thread>
#include <chrono>

namespace obot {

MotorUART::MotorUART(std::string dev_path) {
  dev_path_ = dev_path;
  int result = open();

  if (result < 0) {
    throw std::runtime_error("Error opening " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  // only one item can access uart devices due to protocol
  result = lock();
  if (result < 0) {
    throw std::runtime_error("Error locking: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  set_baud_rate();
  std::cout << "fd " << fd_ << std::endl;
  //motor_txt_ = std::move(std::unique_ptr<SimulatedTextFile>(new SimulatedTextFile()));
  board_name_ = "uart";
  config_ = "uart";
  messages_version_ = MOTOR_MESSAGES_VERSION;
}

void MotorUART::set_timeout_ms(int timeout_ms) {
}

void MotorUART::set_baud_rate(uint32_t baud_rate) {
  struct termios tio;
  int result = tcgetattr(fd_, &tio);
  if (result < 0) {
    throw std::runtime_error("Error tcgetattr: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  tio.c_cflag = PARENB | CS8 | CREAD | CLOCAL;
  tio.c_lflag = 0;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;
  result = cfsetspeed(&tio, baud_rate);
  if (result < 0) {
    throw std::runtime_error("Error cfsetspeed: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  result = tcsetattr(fd_, TCSANOW, &tio);
  if (result < 0) {
    throw std::runtime_error("Error tcsetattr: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
}

ssize_t MotorUART::read() {
  int retval;
  const uint8_t read_command[4] = {0xF0, 0x0F, 3, 0};
  retval = ::write(fd_, read_command, sizeof(read_command));
  if (retval != sizeof(read_command)) {
    read_error_++;
    return retval;
  }
  uint8_t ack_len[3];
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  retval = ::read(fd_, ack_len, sizeof(ack_len));
  if (retval != sizeof(ack_len)) {
    read_error_++;
    return retval;
  }
  std::cout << "ack " << std::hex << +ack_len[0] << " " << +ack_len[1] << " " << +ack_len[2] << 
      " sizeof status " << sizeof(status_) << " read error " << read_error_ << std::dec << std::endl;
  if (ack_len[0] == 0x79 && ack_len[1] == 0x79 && ack_len[2] == 0x3b) {
    Status status_tmp;
    retval = ::read(fd_, &status_tmp, sizeof(status_tmp));
    if (retval != sizeof(status_tmp)) {
      read_error_++;
      return retval;
    }
    retval = ::read(fd_, ack_len, 2);
    if (retval != 2) {
      read_error_++;
      return retval;
    }
    if (ack_len[1] == 0x79) {
      status_ = status_tmp;
      return sizeof(status_);
    }
  }
  read_error_++;
  return 0;
}

ssize_t MotorUART::write() {
    return 0;
}

}; // namespace obot
