#ifndef MOTOR_H
#define MOTOR_H

#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <libgen.h>
#include <libudev.h>

typedef struct {
    uint32_t mcu_timestamp;             // timestamp in microcontroller clock cycles
    uint32_t host_timestamp_received;   // return of host_timestamp from ReceiveData
    float motor_position;               // motor position in radians
    float joint_position;               // joint position in radians
    float iq;                           // Measured motor current in A line-line
    int32_t motor_encoder;              // motor position in raw counts
    float reserved[2];
} Status;

typedef struct {
    uint32_t host_timestamp;            // Value from host
    uint8_t mode_desired;               // 0: open, 1: damped, 2: current, 3: position, 4: velocity, 5: current tuning, 6: position tuning
    float current_desired;              // motor current desired in A line-line
    float position_desired;             // motor position desired in rad
    float velocity_desired;             // motor velocity desired in rad/s
    float reserved;                     // reserved option
} Command;

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
        const char * version = udev_device_get_sysattr_value(dev, "configuration");
        if (version != NULL) {
            version_ = version;
        } else {
            version_ = "";
        }

        udev_device_unref(dev);
        udev_unref(udev);  }
    ~Motor() { close(); }
    int open() { fid_ = ::open(dev_path_.c_str(), O_RDWR); fid_flags_ = fcntl(fid_, F_GETFL); return fid_; }
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
    std::string dev_path() const { return dev_path_; }
    std::string version() const { return version_; }
    int close() { return ::close(fid_); }
    int fd() const { return fid_; }
    const Status *const status() const { return &status_; }
    Command *const command() { return &command_; }
 private:
    int fid_ = 0;
    int fid_flags_;
    std::string serial_number_, name_, dev_path_, base_path_, version_;
    Status status_ = {};
    Command command_ = {};
};

#endif
