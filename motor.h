#ifndef MOTOR_H
#define MOTOR_H

#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <libgen.h>
#include <libudev.h>

struct Status {
	int32_t count;
	int32_t count_received;
	float res[3];
};

struct Command {
	int32_t count;
	uint8_t mode;
	float current_desired;
	float position_desired;
};

class Motor {
 public:
    Motor(std::string dev_path) { dev_path_ = dev_path; 
        struct udev *udev = udev_new();
        struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "usbmisc", basename(const_cast<char *>(dev_path.c_str())));
        const char * name = udev_device_get_sysattr_value(dev, "device/interface");
        if (name != NULL) {
            name_ = name;
        } else {
            name_ = "";
        }

        dev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
        serial_number_ = udev_device_get_sysattr_value(dev, "serial"); 
        base_path_ = basename(const_cast<char *>(udev_device_get_syspath(dev)));

        udev_device_unref(dev);
        udev_unref(udev);  }
    ~Motor() { close(); }
    int open() { fid_ = ::open(dev_path_.c_str(), O_RDWR); fid_flags_ = fcntl(fid_, F_GETFL); };
    ssize_t read() { return ::read(fid_, &status_, sizeof(status_)); };
    ssize_t write() { return ::write(fid_, &command_, sizeof(command_)); };
    ssize_t aread() { int fcntl_error = fcntl(fid_, F_SETFL, fid_flags_ | O_NONBLOCK);
			ssize_t read_error = read(); 
            fcntl_error = fcntl(fid_, F_SETFL, fid_flags_);
            if (read_error != -1) {
                std::cout << "Nonzero aread" << std::endl;
            } else {
                if (errno != EAGAIN) {
                    std::cout << "Aread error " << errno << std::endl;
                }
            }
            return read_error; }
    std::string name() const { return name_; }
    std::string serial_number() const { return serial_number_; }
    std::string base_path() const {return base_path_; }
    int close() { return ::close(fid_); }

    const Status *const status() const { return &status_; }
    Command *const command() { return &command_; }
 private:
    int fid_ = 0;
    int fid_flags_;
    std::string serial_number_, name_, dev_path_, base_path_;
    Status status_ = {};
    Command command_ = {};
};

#endif
