#ifndef MOTOR_H
#define MOTOR_H

#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <memory>

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
#include <time.h>
#include "motor_util_fun.h"
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

namespace obot {

class TextFile {
 public:
    virtual ~TextFile();
    virtual void flush() {}
    virtual ssize_t read(char * /* data */, unsigned int /* length */) { return 0; }
    virtual ssize_t write(const char * /* data */, unsigned int /* length */) { return 0; }
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) { return 0; }
    std::string writeread(const std::string str) {
        char str_in[MAX_API_DATA_SIZE+1];
        ssize_t s = writeread(str.c_str(), str.size(), str_in, MAX_API_DATA_SIZE);
        str_in[s] = 0;
        return str_in;
    }
};

class SimulatedTextFile : public TextFile {
 public:
    virtual ssize_t writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) override  {
        std::string key = data_out;
        std::string value = dict_[key];
        ssize_t len = std::min(length_in, static_cast<unsigned int>(value.size()));
        std::memcpy(data_in, value.c_str(), len);
        return len;
    }
 private:
    std::map<std::string, std::string> dict_ = {{"cpu_frequency", "1000000000"}, {"error_mask", "0"}, {"log", "log end"}, 
        {"fast log", "ok\ntimestamp,"}, {"messages_version", MOTOR_MESSAGES_VERSION}};
    friend class SimulatedMotor;
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
    virtual ~SysfsFile() override;
    virtual void flush() override {
        char c[64];
        while(read(c, 64));
    }
    // locked/blocked to one caller so that a read is a response to write
    virtual ssize_t writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) override {
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
    virtual ssize_t read(char *data, unsigned int length) override {
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
    virtual ssize_t write(const char *data, unsigned int length) override {
        auto retval = ::write(fd_, data, length);
        if (retval < 0) {
            throw std::runtime_error("Sysfs write error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
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
    virtual ~USBFile() override;
    // locked/blocked to one caller so that a read is a response to write
    virtual ssize_t writeread(const char *data_out, unsigned int length_out, char *data_in, unsigned int length_in) override {
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
    ssize_t read(char *data, unsigned int length) override { 
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
    virtual ssize_t write(const char *data, unsigned int length) override { 
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
        char c[MAX_API_DATA_SIZE] = {};
        auto nbytes = motor_txt_->writeread(name_.c_str(), name_.size(), c, MAX_API_DATA_SIZE);
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
class Motor : public MotorDescription {
 public:
    enum MessagesCheck {NONE, MAJOR, MINOR};
    Motor() {}
    Motor(std::string dev_path);
    virtual ~Motor();
    virtual ssize_t read() { return ::read(fd_, &status_, sizeof(status_)); }
    virtual ssize_t write() { if (!no_write_) {
        return ::write(fd_, &command_, sizeof(command_));
     } else {
        std::cerr << "motor " + name() + " locked, not writeable" << std::endl;;
        return 0;
     } }
    virtual int lock() { 
        ::lseek(fd_, 0, SEEK_SET);
        int err = lockf(fd_, F_TLOCK, 0); 
        if (err) {
            std::cerr << "error locking " + name() << std::endl;
        }
        return err;
    }
    virtual int set_nonblock() { nonblock_ = true;
        return fcntl(fd_, F_SETFL, fd_flags_ | O_NONBLOCK); }
    virtual int clear_nonblock() { nonblock_ = false;
        return fcntl(fd_, F_SETFL, fd_flags_ & ~O_NONBLOCK); }
    bool is_nonblocking() const { return nonblock_; }
    virtual ssize_t aread() { int fcntl_error = fcntl(fd_, F_SETFL, fd_flags_ | O_NONBLOCK);
			ssize_t read_error = read(); 
            int fcntl_error2 = fcntl(fd_, F_SETFL, fd_flags_);
            if (fcntl_error < 0 || fcntl_error2 < 0) {
                std::cout << "Aread fcntl error" << std::endl;
            }
            if (read_error != -1) {
                std::cout << "Nonzero aread" << std::endl;
            } else {
                if (errno != EAGAIN) {
                    std::cout << "Aread error " << errno << std::endl;
                }
            }
            return read_error; }
    virtual bool check_messages_version(MessagesCheck check = MAJOR) { 
        if (check == MAJOR) {
            std::string realtime_messages_version = MOTOR_MESSAGES_VERSION;
            std::string motor_messages_version = operator[]("messages_version").get();
            return realtime_messages_version.substr(0, realtime_messages_version.find('.'))
                == motor_messages_version.substr(0, motor_messages_version.find('.'));
        } else if (check == MINOR) {
            return MOTOR_MESSAGES_VERSION == operator[]("messages_version").get();
        } else {
            return true;
        }
    }
    virtual std::string short_version() const override {
        std::string s = version();
        auto pos = std::min(s.find(" "), s.find("-g"));
        return s.substr(0,pos);
    }
    // note will probably not be the final interface
    TextAPIItem operator[](const std::string s) { TextAPIItem t(motor_txt_.get(), s); return t; }
    int fd() const { return fd_; }
    const Status * status() const { return &status_; }
    Command * command() { return &command_; }
    TextFile* motor_text() { return motor_txt_.get(); }
    std::string get_fast_log();
    virtual std::vector<std::string> get_api_options();
    uint32_t get_cpu_frequency() { return std::stoi((*this)["cpu_frequency"].get()); }
 protected:
    int open() { 
        fd_ = ::open(dev_path_.c_str(), O_RDWR); 
        if (lockf(fd_, F_TEST, 0)) {
            no_write_ = true;
        }
        fd_flags_ = fcntl(fd_, F_GETFL); 
        return fd_;
    }
    int close() { return ::close(fd_); }
    int fd_ = 0;
    int fd_flags_;
    bool nonblock_ = false;
    bool no_write_ = false;
    Status status_ = {};
    Command command_ = {};
    std::unique_ptr<TextFile> motor_txt_;
};

class SimulatedMotor : public Motor {
 public:
   SimulatedMotor(std::string name) { 
       name_ = name;
       fd_ = ::open("/dev/zero", O_RDONLY); // so that poll can see something
       motor_txt_ = std::move(std::unique_ptr<SimulatedTextFile>(new SimulatedTextFile()));
       clock_gettime(CLOCK_MONOTONIC, &last_time_);
    }
   virtual ~SimulatedMotor() override;
   virtual ssize_t read() override {  
       timespec time;
       clock_gettime(CLOCK_MONOTONIC, &time);
       double dt = clock_diff(time, last_time_);
       status_.mcu_timestamp += dt*1e9;
       last_time_ = time;
       t_seconds_ += dt;
       switch (active_command_.mode_desired) {
           case VELOCITY:
                status_.motor_position += velocity_*dt;
                status_.joint_position = status_.motor_position/gear_ratio_;
                status_.motor_velocity = velocity_;
                status_.joint_velocity = velocity_/gear_ratio_;
                break;           
           case CURRENT_TUNING:
                status_.iq = active_command_.current_tuning.bias + 
                    active_command_.current_tuning.amplitude*std::sin(active_command_.current_tuning.frequency*2*M_PI*t_seconds_);
           case CURRENT:
                status_.torque = kt_*gear_ratio_*status_.iq;
                // intentional pass through
           case TORQUE:
                velocity_ += status_.torque/inertia_;
                status_.motor_position += velocity_*dt;
                status_.joint_position = status_.motor_position/gear_ratio_;
                status_.motor_velocity = velocity_;
                status_.joint_velocity = velocity_/gear_ratio_;
                break;
       }
       return sizeof(status_);
   }
   virtual ssize_t write() override {
       status_.host_timestamp_received = command_.host_timestamp;
       if (command_.mode_desired == POSITION || command_.mode_desired == CURRENT ||
           command_.mode_desired == TORQUE) {
           status_.iq = command_.current_desired;
       } else {
           status_.iq = 0;
       }
       switch (command_.mode_desired) {
           case POSITION:
               velocity_ = 0;
               status_.motor_position = command_.position_desired;
               status_.joint_position = command_.position_desired/gear_ratio_;
               break;
           case VELOCITY:
               velocity_ = command_.velocity_desired;
               //clock_gettime(CLOCK_MONOTONIC, &last_time_);
               break;
           case TORQUE:
               status_.torque = command_.torque_desired;
               break;
           case CURRENT_TUNING:
               break;
           default:
               break; 
       }
       active_command_ = command_;
       return sizeof(command_); 
   }
   void set_gear_ratio(double gear_ratio) { gear_ratio_ = gear_ratio; }
   virtual std::vector<std::string> get_api_options() override;
 private:
    // a - b in seconds
    double clock_diff(timespec &a, timespec &b) {
        return a.tv_sec - b.tv_sec + a.tv_nsec*1e-9 - b.tv_nsec*1e-9;
    }
    double gear_ratio_ = 1;
    double kt_ = 1;
    double inertia_ = 1;
    double velocity_ = 0;
    double t_seconds_ = 0;
    Command active_command_;
    timespec last_time_;
};

class UserSpaceMotor : public Motor {
 public:
    UserSpaceMotor(std::string dev_path, uint8_t ep_num = 2) { 
        ep_num_ = ep_num;
        aread_transfer_.endpoint |= ep_num;
        dev_path_ = dev_path;
        struct udev *udev = udev_new();
        struct stat st;
        if (stat(dev_path.c_str(), &st) < 0) {
            throw std::runtime_error("Motor stat error " + std::to_string(errno) + ": " + strerror(errno));
        }
        struct udev_device *dev = udev_device_new_from_devnum(udev, 'c', st.st_rdev);
        const char * sysname = udev_device_get_sysname(dev);

        std::string interface_name = sysname;
        interface_name += ":1.0/interface";
        const char * name = udev_device_get_sysattr_value(dev, interface_name.c_str());
        if (name != nullptr) {
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
        if (version != nullptr) {
            version_ = version;
        } else {
            version_ = "";
        }
        devnum_ = std::stoi(udev_device_get_sysattr_value(dev, "devnum"));
        
        udev_device_unref(dev);
        udev_unref(udev);  
        open();
        motor_txt_ = std::move(std::unique_ptr<USBFile>(new USBFile(fd_, 1)));
    }
    virtual ~UserSpaceMotor() override;
    virtual int lock() override {
        std::cerr << "Locking not supported on user space motor" << std::endl;
        errno = 1;
        return -1;
    }
    virtual ssize_t read() override {
        int retval = -1;
        if (!aread_in_progress_) {
            struct usbdevfs_bulktransfer transfer = {
                .ep = static_cast<uint8_t>(ep_num_ | USB_DIR_IN),
                .len = sizeof(status_),
                .timeout = 100,
                .data = &status_
            };

            retval = ::ioctl(fd_, USBDEVFS_BULK, &transfer);
        } else {
            usbdevfs_urb *transfer =  nullptr;
            Timer t(10000000);
            do {
                retval = ::ioctl(fd_, USBDEVFS_REAPURBNDELAY, &transfer);
                if (retval == 0 && transfer->endpoint == static_cast<uint8_t>(ep_num_ | USB_DIR_IN)) {
                    break;
                }
            } while (t.get_time_remaining_ns() && (errno == EAGAIN || retval == 0));
            aread_in_progress_ = false;
        }
        if (retval < 0) {
            throw std::runtime_error("Motor read error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
    virtual ssize_t write() override { 
        struct usbdevfs_urb transfer = {
            .type = USBDEVFS_URB_TYPE_BULK,
            .endpoint = static_cast<uint8_t>(ep_num_ | USB_DIR_OUT),
            .status = 0,
            .flags = 0,
            .buffer = &command_,
            .buffer_length = sizeof(command_),
        };

        int retval = ::ioctl(fd_, USBDEVFS_SUBMITURB, &transfer);
        if (retval < 0) {
            throw std::runtime_error("Motor write error " + std::to_string(errno) + ": " + strerror(errno));
        }
        return retval;
    }
    virtual ssize_t aread() override {
        int retval = ::ioctl(fd_, USBDEVFS_SUBMITURB, &aread_transfer_);
        if (retval < 0) {
            throw std::runtime_error("Motor aread error " + std::to_string(errno) + ": " + strerror(errno));
        }
        aread_in_progress_ = true;
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
    uint8_t ep_num_;
    bool aread_in_progress_ = false;
    usbdevfs_urb aread_transfer_{
        .type = USBDEVFS_URB_TYPE_BULK,
        .endpoint = USB_DIR_IN,
        .status = 0,
        .flags = 0,
        .buffer = &status_,
        .buffer_length = sizeof(status_),
    };
};

}  // namespace obot

#endif
