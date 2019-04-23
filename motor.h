#ifndef MOTOR_H
#define MOTOR_H

#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <libgen.h>
#include <libudev.h>


class Motor {
 public:
    Motor(std::string dev_path) { dev_path_ = dev_path; 
        struct udev *udev = udev_new();
        struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "usbmisc", basename(const_cast<char *>(dev_path.c_str())));
        name_ = udev_device_get_sysattr_value(dev, "device/interface");

        dev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
        serial_number_ = udev_device_get_sysattr_value(dev, "serial"); 

        udev_device_unref(dev);
        udev_unref(udev);  }
    ~Motor() { ::close(fid_); }
    int open() { fid_ = ::open(dev_path_.c_str(), O_RDWR); fid_flags_ = fcntl(fid_, F_GETFL); };
    ssize_t read(void *buf, size_t count) { return ::read(fid_, buf, count); };
    ssize_t write(const void *buf, size_t count) { return ::write(fid_, buf, count); };
    ssize_t aread(void *buf, size_t count) { int fcntl_error = fcntl(fid_, F_SETFL, fid_flags_ | O_NONBLOCK);
			ssize_t read_error = read(buf, count); 
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
 private:
    int fid_ = 0;
    int fid_flags_;
    std::string serial_number_, name_, dev_path_;
};

#endif
