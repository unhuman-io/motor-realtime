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
#include "motor_messages.h"

class TextFile {
 public:
    virtual ~TextFile() {}
    virtual void flush() {}
    virtual ssize_t read(char *data, unsigned int length) { return 0; };
    virtual ssize_t write(const char *data, unsigned int length) { return 0; };
    virtual ssize_t writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) { return 0; }
};

class SysfsFile : public TextFile {
 public:
    SysfsFile (std::string path) {
        path_ = path;
        fd_ = ::open(path_.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("Sysfs open error " + std::to_string(errno) + ": " + strerror(errno) + ", " + path_.c_str());
        }
    }
    ~SysfsFile() {
        int retval = ::close(fd_);
        if (retval) {
            std::cerr << "Sysfs close error " + std::to_string(errno) + ": " + strerror(errno) << std::endl;
        }
    }
    virtual void flush() {
        char c[64];
        while(read(c, 64));
    }
    // locked/blocked to one caller so that a read is a response to write
    ssize_t writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
        ::lseek(fd_, 0, SEEK_SET);
        lockf(fd_, F_LOCK, 0);
        write(data_out, length_out);
        ::lseek(fd_, 0, SEEK_SET);
        auto retval = read(data_in, length_in);
        ::lseek(fd_, 0, SEEK_SET);
        lockf(fd_, F_ULOCK, 0);
        return retval;
    }
    ssize_t read(char *data, unsigned int length) {
        // sysfs file needs to closed and opened or lseek to beginning.
        ::lseek(fd_, 0, SEEK_SET);
        auto retval = ::read(fd_, data, length);

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
        auto retval = ::write(fd_, data, length);
        if (retval < 0) {
            throw std::runtime_error("Sysfs write error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
 private:
    std::string path_;
    int fd_;
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
            .timeout = 1000,
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
    // locked/blocked to one caller so that a read is a response to write
    ssize_t writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) {
        ::lseek(fd_, 0, SEEK_SET);
        lockf(fd_, F_LOCK, 0);
        write(data_out, length_out);
        ::lseek(fd_, 0, SEEK_SET);
        auto retval = read(data_in, length_in);
        ::lseek(fd_, 0, SEEK_SET);
        lockf(fd_, F_ULOCK, 0);
        return retval;
    }
 private:
    unsigned int ep_num_;
    int fd_;
};

class TextAPIItem {
 public:
    TextAPIItem(TextFile *motor_txt, std::string name) : motor_txt_(motor_txt), name_(name) {}

    std::string set(std::string s) {
        std::string s2 = name_ + "=" + s;
        char c[64];
        auto nbytes = motor_txt_->writeread(s2.c_str(), s2.size(), c, 64);
        c[nbytes] = 0;
        return c;
    }
    std::string get() const {        
        char c[64] = {};
        auto nbytes = motor_txt_->writeread(name_.c_str(), name_.size(), c, 64);
        c[nbytes] = 0;
        return c;
    }
    void operator=(const std::string s) {
        set(s);
    }
 private:
    TextFile *motor_txt_;
    std::string name_;
};

inline std::ostream& operator<<(std::ostream& os, TextAPIItem const& item) {
    os << item.get();
    return os;
}
inline double& operator<<(double& d, TextAPIItem const& item) {
    d = std::stod(item.get());
    return d;
}
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
    bool check_messages_version() { return MOTOR_MESSAGES_VERSION == operator[]("messages_version").get(); }
    std::string short_version() const {
        std::string s = version();
        auto pos = s.find("-g");
        return s.substr(0,pos);
    }
    // note will probably not be the final interface
    TextAPIItem operator[](const std::string s) { TextAPIItem t(motor_txt_, s); return t; };
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

class SimulatedMotor : public Motor {
 public:
   SimulatedMotor(std::string name) { name_ = name; motor_txt_ =  new TextFile(); }
   virtual ~SimulatedMotor() {}
   virtual ssize_t read() {
       status_.mcu_timestamp++;
       return sizeof(status_);
   };
   virtual ssize_t write() {
       status_.host_timestamp_received = command_.host_timestamp;
       if (command_.mode_desired == POSITION) {
           status_.motor_position = command_.position_desired;
           status_.joint_position = command_.position_desired/gear_ratio_;
       }
       return sizeof(command_); 
   };
   void set_gear_ratio(double gear_ratio) { gear_ratio_ = gear_ratio; }
 private:
    double gear_ratio_ = 1;
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
