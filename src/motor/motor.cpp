#include "motor.h"

Motor::Motor(std::string dev_path) { 
    dev_path_ = dev_path; 
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

    //dev = udev_device_get_parent(dev);
    const char * parent_dev_path = udev_device_get_devnode(dev);
    motor_txt_ = new USBFile(parent_dev_path, 1);
    udev_device_unref(dev);
    udev_unref(udev);  
}

 Motor::~Motor() { close(); delete motor_txt_; }