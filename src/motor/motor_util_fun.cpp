#include "motor_util_fun.h"
#include <libudev.h>
#include <libgen.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <unistd.h>


namespace obot {

static std::string udev_device_check_and_get_sysattr_value(struct udev_device *dev, const char * name) {
    const char *value = udev_device_get_sysattr_value(dev, name);
    if (value != nullptr) {
        return value;
    }
    return "";
}

std::vector<std::string> udev_list_dfu()
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
    
	/* Create the udev object */
	struct udev *udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		exit(1);
	}
	
	/* Create a list of the devices in the 'hidraw' subsystem. */
	enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_sysattr(enumerate, "idVendor", "0483");
    udev_enumerate_add_match_sysattr(enumerate, "idProduct", "df11");
    
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);

    std::vector<std::string> dev_paths;
	udev_list_entry_foreach(dev_list_entry, devices) {
		// path in /sys/devices/pci*
		const char *path = udev_list_entry_get_name(dev_list_entry);
		struct udev_device *dev = udev_device_new_from_syspath(udev, path);
        const char * devpath = udev_device_get_devpath(dev);

        devpath = udev_device_get_devnode(dev);
        if (devpath) {
            dev_paths.push_back(devpath);
        }
		

		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	return dev_paths;       
}

DFUDevice::DFUDevice(std::string dev_path) {
    dev_path_ = dev_path; 

    struct udev *udev = udev_new();
    struct stat st;
    if (stat(dev_path.c_str(), &st) < 0) {
        throw RuntimeException("Motor stat error " + std::to_string(errno) + ": " + strerror(errno));
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
}

std::string short_status(const std::vector<Status> status) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    for (auto s : status) {
        os << std::setw(6) << s.motor_position << ", ";
    }
    for (auto s : status) {
        os << std::setw(6) << s.joint_position << ", ";
    }
    for (auto s : status) {
        os << std::setw(6) << s.iq << ", ";
    }
    for (auto s : status) {
        os << std::setw(6) << s.torque << ", ";
    }
    return os.str();
}

std::string get_config_dir() {
    std::string config_dir;
    char * config_dir_env = getenv("MOTOR_UTIL_CONFIG_DIR");
    if (config_dir_env == NULL) {
        // right now the only thing in the config directory is the device_ip_map.json
        // will have to figure out the search path implementation later if other files are added
        config_dir = std::string(getenv("HOME")) + "/.config/motor_util/";
        if (access((config_dir + "device_ip_map.json").c_str(), F_OK) != 0) {
            config_dir = "/etc/motor_util/";
            if (access((config_dir + "device_ip_map.json").c_str(), F_OK) != 0) {
                config_dir = "/usr/share/motor-realtime/";
            }
        }
    } else {
        config_dir = std::string(config_dir_env);
        if (config_dir.back() != '/') {
            config_dir += "/";
        }
    }
    return config_dir;
}

}; // namespace obot
