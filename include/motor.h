#ifndef MOTOR_H
#define MOTOR_H

#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <libgen.h>
#include <libudev.h>

// user space driver
#include <linux/usb/ch9.h> // todo why not usb.h
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdexcept>
#include <cstring>

typedef struct {
    uint32_t mcu_timestamp;             // timestamp in microcontroller clock cycles
    uint32_t host_timestamp_received;   // return of host_timestamp from ReceiveData
    float motor_position;               // motor position in radians
    float joint_position;               // joint position in radians
    float iq;                           // Measured motor current in A line-line
    float torque;                       // measured torque in Nm
    int32_t motor_encoder;              // motor position in raw counts
    float reserved[3];
} Status;

enum ModeDesired {OPEN, DAMPED, CURRENT, POSITION, TORQUE, IMPEDANCE, VELOCITY, CURRENT_TUNING, POSITION_TUNING, VOLTAGE, PHASE_LOCK, STEPPER_TUNING, RESET=255};

typedef struct {
    uint32_t host_timestamp;            // Value from host
    uint8_t mode_desired;               // \sa ModeDesired
    float current_desired;              // motor current desired in A line-line
    float position_desired;             // motor position desired in rad
    float velocity_desired;             // motor velocity desired in rad/s
    float torque_desired;
    float reserved;                     // reserved option
} Command;

class TextFile {
 public:
    virtual ~TextFile() {};
    virtual ssize_t read(char *data, unsigned int length) = 0;
    virtual ssize_t write(const char *data, unsigned int length) = 0;
};

class SysfsFile : public TextFile {
 public:
    SysfsFile (std::string path) {
        path_ = path;
    }
    int open() {
        int fd = ::open(path_.c_str(), O_RDWR);
        if (fd >= 0) {
            return fd;
        } else {
            throw std::runtime_error("Sysfs open error " + std::to_string(errno) + ": " + strerror(errno) + ", " + path_.c_str());
        }
    }
    void close(int fd) {
        int retval = ::close(fd);
        if (retval) {
            throw std::runtime_error("Sysfs close error " + std::to_string(errno) + ": " + strerror(errno));
        }
    }
    ssize_t read(char *data, unsigned int length) {
        // sysfs file needs to be opened and closed to read new values
        int fd = open();
        auto retval = ::read(fd, data, length);
        close(fd);
        if (retval < 0) {
            if (errno == ETIMEDOUT) {
                return 0;
            } else {
                throw std::runtime_error("Sysfs read error " + std::to_string(errno) + ": " + strerror(errno));
            }
        }
        return retval;
    }
    ssize_t write(const char *data, unsigned int length) {
        int fd = open();
        auto retval = ::write(fd, data, length);
        close(fd);
        if (retval < 0) {
            throw std::runtime_error("Sysfs write error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
    ~SysfsFile() {}
 private:
    std::string path_;
};

class USBFile : public TextFile {
 public:
    // file fd should be opened already - it can only have one open reference
    USBFile (int fd, uint8_t ep_num = 1) { 
        ep_num_ = ep_num;
        fd_ = fd;
    }
    ssize_t read(char *data, unsigned int length) { 
        struct usbdevfs_bulktransfer transfer = {
            .ep = ep_num_ | USB_DIR_IN,
            .len = length,
            .timeout = 100,
            .data = data
        };

        int retval = ::ioctl(fd_, USBDEVFS_BULK, &transfer);
        if (retval < 0) {
            if (errno == ETIMEDOUT) {
                return 0;
            } else {
                throw std::runtime_error("USB read error " + std::to_string(errno) + ": " + strerror(errno));
            }
        }
        return retval;
    }
    ssize_t write(const char *data, unsigned int length) { 
        char buf[64];
        std::memcpy(buf, data, length);
        struct usbdevfs_bulktransfer transfer = {
            .ep = ep_num_ | USB_DIR_OUT,
            .len = length,
            .timeout = 100,
            .data = buf
        };

        int retval = ::ioctl(fd_, USBDEVFS_BULK, &transfer);
        if (retval < 0) {
            throw std::runtime_error("USB write error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
 private:
    unsigned int ep_num_;
    int fd_;
};
class Motor {
 public:
    Motor() {}
    Motor(std::string dev_path);
    virtual ~Motor();
    virtual ssize_t read() { return ::read(fd_, &status_, sizeof(status_)); };
    virtual ssize_t write() { return ::write(fd_, &command_, sizeof(command_)); };
    ssize_t aread() { int fcntl_error = fcntl(fd_, F_SETFL, fd_flags_ | O_NONBLOCK);
			ssize_t read_error = read(); 
            fcntl_error = fcntl(fd_, F_SETFL, fd_flags_);
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
    int fd() const { return fd_; }
    const Status *const status() const { return &status_; }
    Command *const command() { return &command_; }
    TextFile* motor_text() { return motor_txt_; }
 protected:
    int open() { fd_ = ::open(dev_path_.c_str(), O_RDWR); fd_flags_ = fcntl(fd_, F_GETFL); return fd_; }
    int close() { return ::close(fd_); }
    int fd_ = 0;
    int fd_flags_;
    std::string serial_number_, name_, dev_path_, base_path_, version_;
    Status status_ = {};
    Command command_ = {};
    TextFile *motor_txt_;
};

class UserSpaceMotor : public Motor {
 public:
    UserSpaceMotor(std::string dev_path, uint8_t ep_num = 2) { 
        ep_num_ = ep_num;
        dev_path_ = dev_path; 
        struct udev *udev = udev_new();
        struct stat st;
        if (stat(dev_path.c_str(), &st) < 0) {
            throw std::runtime_error("Motor stat error " + std::to_string(errno) + ": " + strerror(errno));
        }
        struct udev_device *dev = udev_device_new_from_devnum(udev, 'c', st.st_rdev);
                const char * sysname = udev_device_get_sysname(dev);
        const char * subsystem = udev_device_get_subsystem(dev);
        const char * devpath = udev_device_get_devpath(dev);
        //struct udev_device *dev = udev_device_new_from_syspath(udev, syspath;
       // struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "usb", sysname);
        std::string interface_name = sysname;
        interface_name += ":1.0/interface";
        const char * name = udev_device_get_sysattr_value(dev, interface_name.c_str());
        if (name != NULL) {
            name_ = name;
        } else {
            name_ = "";
        }

        // dev = udev_device_get_parent_with_subsystem_devtype(
		//        dev,
		//        "usb",
		//        "usb_device");
        serial_number_ = udev_device_get_sysattr_value(dev, "serial"); 
        base_path_ = basename(const_cast<char *>(udev_device_get_syspath(dev)));
        const char * version = udev_device_get_sysattr_value(dev, "configuration");
        if (version != NULL) {
            version_ = version;
        } else {
            version_ = "";
        }
        
        udev_device_unref(dev);
        udev_unref(udev);  
        open();
        motor_txt_ = new USBFile(fd_, 1);
    }
    virtual ~UserSpaceMotor() { close(); }
    virtual ssize_t read() { 
        char data[64];
        struct usbdevfs_bulktransfer transfer = {
            .ep = ep_num_ | USB_DIR_IN,
            .len = sizeof(status_),
            .timeout = 100,
            .data = &status_
        };

        int retval = ::ioctl(fd_, USBDEVFS_BULK, &transfer);
        if (retval < 0) {
            throw std::runtime_error("Motor read error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
    virtual ssize_t write() { 
        char data[64];
        struct usbdevfs_bulktransfer transfer = {
            .ep = ep_num_ | USB_DIR_OUT,
            .len = sizeof(command_),
            .timeout = 100,
            .data = &command_
        };

        int retval = ::ioctl(fd_, USBDEVFS_BULK, &transfer);
        if (retval < 0) {
            throw std::runtime_error("Motor write error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
 private:
    int open() {
        int retval = Motor::open();
        struct usbdevfs_disconnect_claim claim = { 0, USBDEVFS_DISCONNECT_CLAIM_IF_DRIVER, "usb_rt" };
        int ioval = ::ioctl(fd_, USBDEVFS_DISCONNECT_CLAIM, &claim); // will take control from driver if one is installed
        if (ioval < 0) {
            throw std::runtime_error("Motor open error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
    int close() {
        int interface_num = 0;
        int ioval = ::ioctl(fd_, USBDEVFS_RELEASEINTERFACE, &interface_num); 
        if (ioval < 0) {
            throw std::runtime_error("Motor release interface error " + std::to_string(errno) + ": " + strerror(errno));
        }
        struct usbdevfs_ioctl connect = { .ifno = 0, .ioctl_code=USBDEVFS_CONNECT };
        ioval = ::ioctl(fd_, USBDEVFS_IOCTL, &connect); // allow kernel driver to reconnect
        if (ioval < 0) {
            throw std::runtime_error("Motor close error " + std::to_string(errno) + ": " + strerror(errno));
        }
        // fd_ closed by base
        return 0;
    }
    unsigned int ep_num_;
};

#endif
