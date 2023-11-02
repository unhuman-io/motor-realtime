#include "motor.h"

namespace obot {

static std::string udev_device_check_and_get_sysattr_value(struct udev_device *dev, const char * name) {
    const char *value = udev_device_get_sysattr_value(dev, name);
    if (value != nullptr) {
        return value;
    }
    return "";
}

Motor::Motor(std::string dev_path) { 
    dev_path_ = dev_path; 
    struct udev *udev = udev_new();
    struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "usbmisc", basename(const_cast<char *>(dev_path.c_str())));
    if (!dev) {
        throw std::runtime_error("No device: " + dev_path);
    }
    name_ = udev_device_check_and_get_sysattr_value(dev, "device/interface");

    std::string text_api_path = udev_device_get_syspath(dev);
    attr_path_ = text_api_path + "/device";
    motor_txt_ = std::move(std::unique_ptr<SysfsFile>(new SysfsFile(text_api_path + "/device/text_api")));

    struct udev_device *dev_parent = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device");
    serial_number_ = udev_device_check_and_get_sysattr_value(dev_parent, "serial"); 
    base_path_ = basename(const_cast<char *>(udev_device_get_syspath(dev_parent)));
    version_ = udev_device_check_and_get_sysattr_value(dev_parent, "configuration");
    devnum_ = std::stoi(udev_device_check_and_get_sysattr_value(dev_parent, "devnum"));

    udev_device_unref(dev);
    udev_unref(udev);
    open();
}

Motor::~Motor() { close(); }

std::string Motor::get_fast_log() {
    std::string s_read;
    do {
        s_read = motor_txt_->writeread("log");
    } while (s_read != "log end");
    s_read = motor_txt_->writeread("fast_log");
    std::string s_log;
    do {
        s_read = motor_txt_->writeread("log");
        size_t index = s_read.find(") ");
        if (s_read != "log end" && 
            s_read != "ok") {
            if (index == std::string::npos) {
                s_log += s_read + '\n';
            } else {
                // eliminate leading timing information in parentheses
                s_log += s_read.substr(index+2, std::string::npos) + '\n';
            }
        }
    } while (s_read != "log end");
    return s_log;
}

std::vector<std::string> Motor::get_api_options() {
    std::vector<std::string> v;
    uint16_t length = std::stoi((*this)["api_length"].get());
    for (uint16_t i=0; i<length; i++) {
        v.push_back((*this)["api_name=" + std::to_string(i)].get());
    }
    return v;
}

std::vector<std::string> SimulatedMotor::get_api_options() {
    std::vector<std::string> v;
    std::map<std::string, std::string> &dict = static_cast<SimulatedTextFile*>(motor_txt_.get())->dict_;
    for(std::map<std::string, std::string>::iterator it = dict.begin(); it != dict.end(); ++it) {
        v.push_back(it->first);
    }
    return v;
}

void Motor::set_timeout_ms(int timeout_ms) {
    std::string timeout_path = attr_path_ + "/timeout_ms";
    int fd = ::open(timeout_path.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("timeout_ms open error " + std::to_string(errno) + ": " + strerror(errno) + ", " + timeout_path);
    }
    std::string s = std::to_string(timeout_ms);
    int retval = ::write(fd, s.c_str(), s.size());
    if (retval < 0) {
        throw std::runtime_error("set timeout error " + std::to_string(errno) + ": " + strerror(errno));
    }
    ::close(fd);
}

int Motor::get_timeout_ms() const {
    std::string timeout_path = attr_path_ + "/timeout_ms";
    int fd = ::open(timeout_path.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("timeout_ms open error " + std::to_string(errno) + ": " + strerror(errno) + ", " + timeout_path);
    }
    char c[64];
    int retval = ::read(fd, c, 64);
    if (retval < 0) {
        throw std::runtime_error("get timeout error " + std::to_string(errno) + ": " + strerror(errno));
    }
    ::close(fd);
    return std::atoi(c);
}

TextFile::~TextFile() {}

SysfsFile::~SysfsFile() {
    int retval = ::close(fd_);
    if (retval) {
        std::cerr << "Sysfs close error " + std::to_string(errno) + ": " + strerror(errno) << std::endl;
    }
}

USBFile::~USBFile() {}

UserSpaceMotor::~UserSpaceMotor() { close(); }

SimulatedMotor::~SimulatedMotor() { ::close(fd_); }

}  // namespace obot
