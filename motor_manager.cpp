#include "motor_manager.h"
#include "motor.h"

#include <libudev.h>

#include <cstring>

// Returns a vector of strings that contain the dev file locations,
// e.g. /dev/skel0
std::vector<std::string> udev (void)
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
	udev_enumerate_add_match_sysname(enumerate, "skel*");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);

    std::vector<std::string> dev_paths;
	udev_list_entry_foreach(dev_list_entry, devices) {
		// path in /sys/devices/pci*
		const char *path = udev_list_entry_get_name(dev_list_entry);
		struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        // TODO better way of identifying other than interface number 0
		if (std::string("00") == udev_device_get_sysattr_value(dev, "device/bInterfaceNumber")) {
			const char * devpath = udev_device_get_devnode(dev);
            dev_paths.push_back(devpath);
		}
		

		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	return dev_paths;       
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_connected_motors() {
    auto dev_paths = udev();
    std::vector<std::shared_ptr<Motor>> m;
    for (auto dev_path : dev_paths) {
        m.push_back(std::make_shared<Motor>(dev_path));
    }
    
    return m;
}