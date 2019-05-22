#include "motor_manager.h"
#include "motor.h"
#include <iostream>
#include <iomanip>

int main() {
    MotorManager m;
    auto motors = m.get_connected_motors();
    int name_width = 10;
    int serial_number_width = 15;
    int version_width = 60;
    int path_width = 15;
    std::cout << std::setw(name_width) << "Name" << std::setw(serial_number_width) << " Serial number" 
                << std::setw(version_width) << "Version" << std::setw(path_width) << "Path" << std::endl;
    std::cout << std::setw(name_width + serial_number_width + version_width + path_width) << std::setfill('-') << "" << std::setfill(' ') << std::endl;
    for (auto m : motors) {
        std::cout << std::setw(name_width) << m->name()
                  << std::setw(serial_number_width) << m->serial_number() 
                  << std::setw(version_width) << m->version()
                  << std::setw(path_width) << m->base_path() << std::endl;
    }
}