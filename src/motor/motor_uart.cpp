#include "motor_uart.h"
#include <errno.h>
#include "poll.h"
#include <asm/termbits.h>

#include <thread>
#include <chrono>

namespace obot {

MotorUARTRaw::MotorUARTRaw(std::string dev_path, uint32_t baud_rate) {
  dev_path_ = dev_path;
  int result = open();
  if (result < 0) {
    throw std::runtime_error("Error opening " + dev_path_ + " error " + std::to_string(errno) + ": " + strerror(errno));
  }
  realtime_mailbox_.fd_ = fd_;
  motor_txt_ = std::move(std::unique_ptr<Mailbox>(new Mailbox()));
  Mailbox * text_mailbox = static_cast<Mailbox *>(motor_txt_.get());
  text_mailbox->fd_ = fd_;
  text_mailbox->send_mailbox_ = 4;
  text_mailbox->recv_mailbox_ = 4;
  text_mailbox->send_recv_mailbox_ = 4;
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

void MotorUARTRaw::set_timeout_ms(int timeout_ms) {
}

void MotorUARTRaw::set_baud_rate(uint32_t baud_rate) {
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

ssize_t MotorUARTRaw::read() {
  //static int count = 0;
  ssize_t result = realtime_mailbox_.read((char *) &status_, sizeof(status_));
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

ssize_t MotorUARTRaw::write() {
  return realtime_mailbox_.write((char *) &command_, sizeof(command_));
}

ssize_t Mailbox::read(char * data, unsigned int length) {
  int retval;
  std::this_thread::sleep_for(std::chrono::microseconds(400));
  const uint8_t read_command[1] = {recv_mailbox_};
  retval = ::write(fd_, read_command, sizeof(read_command));
  if (retval != sizeof(read_command)) {
    read_error_++;
    return retval;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  retval = ::read(fd_, data, length);
  // std::cout << "read " << retval << std::endl;
  if (retval < length) {
    read_error_++;
    //std::cout << "read error " << retval << std::endl;
    return retval;
  }
  return retval;
}

ssize_t Mailbox::write(const char * data, unsigned int length) {
  int retval;
  std::this_thread::sleep_for(std::chrono::microseconds(400));
  uint8_t write_command[1+length];
  write_command[0] = send_mailbox_;
  std::memcpy(&write_command[1], data, length);
  retval = ::write(fd_, write_command, length+1);
  if (retval != length+1) {
    write_error_++;
    return retval;
  }
  return length;
}

ssize_t Mailbox::writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
  uint8_t write_command[1+length_out];
  write_command[0] = send_recv_mailbox_;
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  std::memcpy(&write_command[1], data_out, length_out);
  int retval = ::write(fd_, write_command, length_out+1);
  if (retval != length_out+1) {
    write_error_++;
    return retval;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  retval = ::read(fd_, data_in, length_in);
  // std::cout << "read in" << retval << std::endl;
  if (retval < 0) {
    read_error_++;
    return retval;
  }
  return retval;
}

}; // namespace obot
