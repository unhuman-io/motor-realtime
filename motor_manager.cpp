#include "motor_manager.h"
#include "motor.h"

#include <libudev.h>

#include <cstring>
#include <algorithm>

// Returns a vector of strings that contain the dev file locations,
// e.g. /dev/skel0
static std::vector<std::string> udev (void)
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
    
    motors_ = m;
    return motors_;
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_motors_by_name(std::vector<std::string> names) {
    auto connected_motors = get_connected_motors();
    std::vector<std::shared_ptr<Motor>> m(names.size());
    for (int i=0; i<names.size(); i++) {
        std::vector<std::shared_ptr<Motor>> found_motors;
        std::copy_if(connected_motors.begin(), connected_motors.end(), std::back_inserter(found_motors), [&names, &i](std::shared_ptr<Motor> m){ return names[i] == m->name(); });
        if (found_motors.size() == 1) {
            m[i] = found_motors[1];
        } else {
            std::cout << "Error: found " << found_motors.size() << " motors matching \"" << names[i] << "\"" << std::endl;
        }
    }
    motors_ = m;
    return motors_;
}

void MotorManager::open() {
    for (auto m : motors_) {
        m->open();
    }
}

std::vector<Status> MotorManager::read() {
    std::vector<Status> statuses(motors_.size());
    for (int i=0; i<motors_.size(); i++) {
        motors_[i]->read();
        statuses[i] = *motors_[i]->status();
    }
    return statuses;
}

void MotorManager::write(std::vector<Command> commands) {
    for (int i=0; i<motors_.size(); i++) {
        *motors_[i]->command() = commands[i];
        motors_[i]->write();
    }
}

void MotorManager::aread() {
    for (int i=0; i<motors_.size(); i++) {
        motors_[i]->aread();
    }
}

void MotorManager::close() {
    for (auto m : motors_) {
        m->close();
    }
}