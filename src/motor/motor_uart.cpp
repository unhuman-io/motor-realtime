#include "motor_uart.h"
#include <errno.h>
#include "poll.h"
#include <termios.h>

#include <thread>
#include <chrono>

namespace obot {

MotorUART::MotorUART(std::string dev_path, uint32_t baud_rate) {
  dev_path_ = dev_path;
  int result = open();
  if (result < 0) {
    throw std::runtime_error("Error opening " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  realtime_mailbox_.send_mailbox_ = 4;
  realtime_mailbox_.recv_mailbox_ = 3;
  realtime_mailbox_.fd_ = fd_;
  pollfds_[0].fd = fd_;
  pollfds_[0].events = POLLIN;
  motor_txt_ = std::move(std::unique_ptr<Mailbox>(new Mailbox()));
  Mailbox * text_mailbox = static_cast<Mailbox *>(motor_txt_.get());
  text_mailbox->fd_ = fd_;
  // only one item can access uart devices due to protocol
  result = lock();
  if (result < 0) {
    throw std::runtime_error("Error locking: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  set_baud_rate(baud_rate);
  if (sync() < 0) {
    throw std::runtime_error("Error syncing: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  
  version_ = operator[]("version").get();
  messages_version_ = operator[]("messages_version").get();
  board_name_ = operator[]("board_name").get();
  board_rev_ = operator[]("board_rev").get();
  board_num_ = operator[]("board_num").get();
  config_ = operator[]("config").get();
  serial_number_ = operator[]("serial").get();
  name_ = operator[]("name").get();
}

void MotorUART::set_timeout_ms(int timeout_ms) {
}

void MotorUART::set_baud_rate(uint32_t baud_rate) {
  struct termios tio;
  int result;
  result = tcflush(fd_, TCIOFLUSH);
  if (result < 0) {
    throw std::runtime_error("Error tcflush: " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  result = tcgetattr(fd_, &tio);
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
  return realtime_mailbox_.read((char *) &status_, sizeof(status_));
}

ssize_t MotorUART::write() {
  return realtime_mailbox_.write((char *) &command_, sizeof(command_));
}

int MotorUART::sync() {
  uint8_t dummy[1] = {0x7F};
  uint8_t ack[1] = {};
  int retval;
  // maximum tries of 256 due to protocol
  for (int i=0; i<256; i++) {
    retval = ::write(fd_, dummy, sizeof(dummy));
    if (retval != sizeof(dummy)) {
      sync_error_++;
      return retval;
    }
    timespec ts = {};
    ts.tv_nsec = 400*1000;
    retval = ppoll(pollfds_, 1, &ts, nullptr);
    if (retval < 0) {
      sync_error_++;
      return retval;
    } else if (retval > 0) {
      retval = ::read(fd_, ack, sizeof(ack));
      if (retval != sizeof(ack)) {
        sync_error_++;
        return retval;
      } else {
        return 0;
      }
    }
  }
  errno = EIO;
  return -1;
}

ssize_t Mailbox::read(char * data, unsigned int length) {
  int retval;
  const uint8_t read_command[4] = {0xF0, 0x0F, recv_mailbox_, 0};
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
  // std::cout << "ack " << std::hex << +ack_len[0] << " " << +ack_len[1] << " " << +ack_len[2] 
  //   << " read error " << read_error_ << std::dec << std::endl;
  if (ack_len[0] == 0x79 && ack_len[1] == 0x79) {
    length = ack_len[2] + 1;
    retval = ::read(fd_, data, length);
    if (retval != length) {
      read_error_++;
      return retval;
    }
    retval = ::read(fd_, ack_len, 2);
    if (retval != 2) {
      read_error_++;
      return retval;
    }
    if (ack_len[1] == 0x79) {
      return length;
    }
  }
  read_error_++;
  return 0;
}

ssize_t Mailbox::write(const char * data, unsigned int length) {
  int retval;
  const uint8_t write_command[5] = {0xF1, 0x0E, send_mailbox_, (uint8_t) (length-1), 0};
  retval = ::write(fd_, write_command, sizeof(write_command));
  if (retval != sizeof(write_command)) {
    write_error_++;
    return retval;
  }
  uint8_t ack_len[2];
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  retval = ::read(fd_, ack_len, sizeof(ack_len));
  if (retval != sizeof(ack_len)) {
    write_error_++;
    return retval;
  }
  retval = ::write(fd_, data, length);
  if (retval != length) {
    write_error_++;
    return retval;
  }
  uint8_t checksum[1] = {};
  retval = ::write(fd_, checksum, sizeof(checksum));
  if (retval != sizeof(checksum)) {
    write_error_++;
    return retval;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  retval = ::read(fd_, ack_len, 1);
  if (retval != 1) {
    write_error_++;
    return retval;
  }
  return length;
}

ssize_t Mailbox::writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
  int write_result = write(data_out, length_out);
  if (write_result < 0) {
    return write_result;
  }
  return read(data_in, length_in);
}

}; // namespace obot
