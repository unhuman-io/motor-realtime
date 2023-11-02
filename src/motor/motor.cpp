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
    std::string s_read, s_out;
    s_out += "timestamp, position, iq_des, iq_meas_filt, ia, ib, ic, va, vb, vc, vbus\n";

    for(int j=0; j<4; j++) {
        s_read = motor_txt_->writeread("fast_log");
        for(int i=0; i<22; i++) {
            FastLog log = *(FastLog *) (s_read.c_str() + i*sizeof(FastLog));
            s_out += 
                std::to_string(log.timestamp) + ", " +
                std::to_string(log.measured_motor_position) + ", " +
                std::to_string(log.command_iq) + ", " +
                std::to_string(log.measured_iq) + ", " +
                std::to_string(log.measured_ia) + ", " +
                std::to_string(log.measured_ib) + ", " +
                std::to_string(log.measured_ic) + ", " +
                std::to_string(log.command_va) + ", " +
                std::to_string(log.command_vb) + ", " +
                std::to_string(log.command_vc) + ", " +
                std::to_string(log.vbus) + "\n";
        }
    }
    return s_out;
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
