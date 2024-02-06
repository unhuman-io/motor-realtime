#include "motor_usb.h"

namespace obot {

static std::string udev_device_check_and_get_sysattr_value(struct udev_device* dev,
                                                           const char* name) {
    const char* value = udev_device_get_sysattr_value(dev, name);
    if (value != nullptr) {
        return value;
    }
    return "";
}

static int get_lock_pid(int fd, pid_t* pid) {
    struct flock lock;
    lock.l_type = F_WRLCK;
    lock.l_start = 0;
    lock.l_whence = 0;
    lock.l_len = 0;
    int err = ::fcntl(fd, F_GETLK, &lock);
    if (err == 0) {
        *pid = lock.l_pid;
    }
    return err;
}

MotorUSB::~MotorUSB() { close(); }

MotorUSB::MotorUSB(std::string dev_path) {
    dev_path_ = dev_path;
    struct udev* udev = udev_new();
    struct udev_device* dev = udev_device_new_from_subsystem_sysname(
        udev, "usbmisc", basename(const_cast<char*>(dev_path.c_str())));
    if (!dev) {
        throw std::runtime_error("No device: " + dev_path);
    }
    name_ = udev_device_check_and_get_sysattr_value(dev, "device/interface");

    std::string text_api_path = udev_device_get_syspath(dev);
    attr_path_ = text_api_path + "/device";
    motor_txt_ =
        std::move(std::unique_ptr<SysfsFile>(new SysfsFile(text_api_path + "/device/text_api")));

    struct udev_device* dev_parent =
        udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    serial_number_ = udev_device_check_and_get_sysattr_value(dev_parent, "serial");
    base_path_ = basename(const_cast<char*>(udev_device_get_syspath(dev_parent)));
    version_ = udev_device_check_and_get_sysattr_value(dev_parent, "configuration");
    devnum_ = std::stoi(udev_device_check_and_get_sysattr_value(dev_parent, "devnum"));

    udev_device_unref(dev);
    udev_unref(udev);
    open();

    messages_version_ = operator[]("messages_version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
}

ssize_t MotorUSB::read() { return ::read(fd_, &status_, sizeof(status_)); }

ssize_t MotorUSB::write() {
    if (!no_write_) {
        return ::write(fd_, &command_, sizeof(command_));
    } else {
        std::cerr << "motor " + name() + " locked";
        pid_t pid;
        int err = get_lock_pid(fd_, &pid);
        if (err == 0) {
            std::cerr << " by process: " << pid;
        }
        std::cerr << ", not writeable" << std::endl;
        return -1;
    }
}

int MotorUSB::lock() {
    ::lseek(fd_, 0, SEEK_SET);
    int err = lockf(fd_, F_TLOCK, 0);
    if (err) {
        std::cerr << "error locking " + name();
        pid_t pid;
        int err2 = get_lock_pid(fd_, &pid);
        if (err2 == 0) {
            std::cerr << ", already locked by process: " << pid;
        }
        std::cerr << std::endl;
    }
    return err;
}

int MotorUSB::set_nonblock() {
    nonblock_ = true;
    return fcntl(fd_, F_SETFL, fd_flags_ | O_NONBLOCK);
}

int MotorUSB::clear_nonblock() {
    nonblock_ = false;
    return fcntl(fd_, F_SETFL, fd_flags_ & ~O_NONBLOCK);
}

ssize_t MotorUSB::aread() {
    int fcntl_error = fcntl(fd_, F_SETFL, fd_flags_ | O_NONBLOCK);
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
    return read_error;
}

int MotorUSB::open() {
    fd_ = ::open(dev_path_.c_str(), O_RDWR);
    if (lockf(fd_, F_TEST, 0)) {
        no_write_ = true;
    }
    fd_flags_ = fcntl(fd_, F_GETFL);
    return fd_;
}

int MotorUSB::close() {
    if (fd_ != -1) {
        return ::close(fd_);
    }
    return 0;
}

}  // namespace obot
