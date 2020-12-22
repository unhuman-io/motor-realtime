#include "motor_manager.h"
#include "motor.h"

#include <libudev.h>

#include <cstring>
#include <algorithm>
#include <poll.h>
#include <sstream>
#include <thread>
#include <chrono>

// Returns a vector of strings that contain the dev file locations,
// e.g. /dev/skel0
static std::vector<std::string> udev (bool user_space_driver=false)
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
    if (user_space_driver) {
        udev_enumerate_add_match_sysattr(enumerate, "idVendor", "3293");
        udev_enumerate_add_match_sysattr(enumerate, "idProduct", "0100");
    } else {
        udev_enumerate_add_match_sysname(enumerate, "usbrt*");
        udev_enumerate_add_match_sysname(enumerate, "mtr*");
    }
    
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);

    std::vector<std::string> dev_paths;
	udev_list_entry_foreach(dev_list_entry, devices) {
		// path in /sys/devices/pci*
		const char *path = udev_list_entry_get_name(dev_list_entry);
		struct udev_device *dev = udev_device_new_from_syspath(udev, path);
        const char * sysname = udev_device_get_sysname(dev);
        const char * subsystem = udev_device_get_subsystem(dev);
        const char * devpath = udev_device_get_devpath(dev);

        // TODO better way of identifying other than interface number 0
		//if (std::string("00") == udev_device_get_sysattr_value(dev, "device/bInterfaceNumber")) {
			devpath = udev_device_get_devnode(dev);
            dev_paths.push_back(devpath);
		//}
		

		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	return dev_paths;       
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_connected_motors() {
    auto dev_paths = udev(user_space_driver_);
    std::vector<std::shared_ptr<Motor>> m;
    for (auto dev_path : dev_paths) {
        if (user_space_driver_ == true) {
            m.push_back(std::make_shared<UserSpaceMotor>(dev_path));
        } else {
             m.push_back(std::make_shared<Motor>(dev_path));
        }
    }
    
    motors_ = m;
    commands_.resize(m.size());
    return motors_;
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_motors_by_name_function(std::vector<std::string> names, std::string (Motor::*name_fun)() const ) {
    auto connected_motors = get_connected_motors();
    std::vector<std::shared_ptr<Motor>> m(names.size());
    for (int i=0; i<names.size(); i++) {
        std::vector<std::shared_ptr<Motor>> found_motors;
        std::copy_if(connected_motors.begin(), connected_motors.end(), std::back_inserter(found_motors), [&names, &i, &name_fun](std::shared_ptr<Motor> m){ return names[i] == (m.get()->*name_fun)(); });
        if (found_motors.size() == 1) {
            m[i] = found_motors[0];
        } else {
            std::cout << "Error: found " << found_motors.size() << " motors matching \"" << names[i] << "\"" << std::endl;
        }
    }
    motors_ = m;
    commands_.resize(m.size());
    return motors_;
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_motors_by_name(std::vector<std::string> names) {
    return get_motors_by_name_function(names, &Motor::name);
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_motors_by_serial_number(std::vector<std::string> serial_numbers) {
    return get_motors_by_name_function(serial_numbers, &Motor::serial_number);
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_motors_by_path(std::vector<std::string> paths) {
    return get_motors_by_name_function(paths, &Motor::base_path);
}

std::vector<std::shared_ptr<Motor>> MotorManager::get_motors_by_devpath(std::vector<std::string> devpaths) {
    return get_motors_by_name_function(devpaths, &Motor::dev_path);
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
    count_++;
    if (auto_count_) {
        set_command_count(count_);
        for (int i=0; i<commands.size(); i++) {
            commands[i].host_timestamp = count_;
        }
    }
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

void MotorManager::set_commands(std::vector<Command> commands) {
    for (int i=0; i<commands_.size(); i++) {
        commands_[i] = commands[i];
    }
}

void MotorManager::set_command_count(int32_t count) {
    for (auto &c : commands_) {
        c.host_timestamp = count;
    }
}

void MotorManager::set_command_mode(uint8_t mode) {
    for (auto &c : commands_) {
        c.mode_desired = mode;
    }
}
    
void MotorManager::set_command_current(std::vector<float> current) {
    for (int i=0; i<commands_.size(); i++) {
        commands_[i].current_desired = current[i];
    }
}

void MotorManager::set_command_position(std::vector<float> position) {
    for (int i=0; i<commands_.size(); i++) {
        commands_[i].position_desired = position[i];
    }
}

void MotorManager::set_command_velocity(std::vector<float> velocity) {
    for (int i=0; i<commands_.size(); i++) {
        commands_[i].velocity_desired = velocity[i];
    }
}

void MotorManager::set_command_torque(std::vector<float> torque) {
    for (int i=0; i<commands_.size(); i++) {
        commands_[i].torque_desired = torque[i];
    }
}

void MotorManager::set_command_reserved(std::vector<float> reserved) {
    for (int i=0; i<commands_.size(); i++) {
        commands_[i].reserved = reserved[i];
    }
}

void MotorManager::write_saved_commands() {
    write(commands_);
}

int MotorManager::serialize_command_size() const {
    return sizeof(commands_.size()) + commands_.size() * sizeof(commands_[0]);
}

int MotorManager::serialize_saved_commands(char *data) const {
    char *datastart = data;
    auto size = commands_.size();
    std::memcpy(data, &size, sizeof(size));
    data += sizeof(size);
    for (int i=0; i<commands_.size(); i++) {
        std::memcpy(data, &commands_[i], sizeof(commands_[0]));
        data += sizeof(commands_[0]);
    }
    return data - datastart;
}

bool MotorManager::deserialize_saved_commands(char *data) {
   auto size = commands_.size();
   decltype(commands_.size()) size_in;
   std::memcpy(&size_in, data, sizeof(size_in));
   if (size_in == size) {
        data += sizeof(size_in);
        for (int i=0; i<commands_.size(); i++) {
            std::memcpy(&commands_[i], data, sizeof(commands_[0]));
            data += sizeof(commands_[0]);
        }
        return true;
   }
   return false;
}

int MotorManager::poll() {
    auto pollfds = new pollfd[motors_.size()];
    for (int i=0; i<motors_.size(); i++) {
        pollfds[i].fd = motors_[i]->fd();
        pollfds[i].events = POLLIN;
    }
    int retval = ::poll(pollfds, motors_.size(), 1);
    delete [] pollfds;
    return retval;
}

std::string MotorManager::command_headers() const {
    std::stringstream ss;
    int length = motors_.size();
    for (int i=0;i<length;i++) {
        ss << "host_timestamp" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "mode_desired" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "current_desired" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "position_desired" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "velocity_desired" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "reserved" << i << ", ";
    }
    return ss.str();
}

std::string MotorManager::status_headers() const {
    std::stringstream ss;
    int length = motors_.size();
    for (int i=0;i<length;i++) {
        ss << "mcu_timestamp" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "host_timestamp_received" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "motor_position" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "joint_position" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "iq" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "torque" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "motor_encoder" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "reserved0" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "reserved1" << i << ", ";
    }
    for (int i=0;i<length;i++) {
        ss << "reserved2" << i << ", ";
    }
    return ss.str();
}

std::vector<std::vector<Status>> MotorManager::collect_data(double t_seconds, double frequency_hz) {
    auto dt = std::chrono::nanoseconds((uint64_t) (1e9/frequency_hz));
    auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    std::vector<std::vector<Status>> status;
    do {
        next_time += dt;
        poll();
        status.push_back(read());
        std::this_thread::sleep_until(next_time);
    } while ((next_time - start_time) < std::chrono::nanoseconds((uint64_t) (t_seconds*1e9)));
    return status;
}
