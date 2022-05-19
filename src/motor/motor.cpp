#include "motor.h"

static std::string udev_device_check_and_get_sysattr_value(struct udev_device *dev, const char * name) {
    const char *value = udev_device_get_sysattr_value(dev, name);
    if (value != NULL) {
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
    motor_txt_ = new SysfsFile(text_api_path + "/device/text_api");

    struct udev_device *dev_parent = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device");
    serial_number_ = udev_device_check_and_get_sysattr_value(dev_parent, "serial"); 
    base_path_ = basename(const_cast<char *>(udev_device_get_syspath(dev_parent)));
    version_ = udev_device_check_and_get_sysattr_value(dev_parent, "configuration");
    
    udev_device_unref(dev);
    udev_unref(udev);
    open();
}

 Motor::~Motor() { close(); delete motor_txt_; }