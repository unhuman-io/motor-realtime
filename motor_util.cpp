#include "motor_manager.h"
#include "motor.h"
#include <iostream>
#include <iomanip>

int main() {
    MotorManager m;
    auto motors = m.get_connected_motors();
    int name_width = 10;
    int serial_number_width = 15;
    std::cout << std::setw(name_width) << "Name" << std::setw(serial_number_width) << " Serial number" << std::endl;
    std::cout << std::setw(name_width + serial_number_width) << std::setfill('-') << "" << std::setfill(' ') << std::endl;
    for (auto m : motors) {
        std::cout << std::setw(name_width) << m->name() << std::setw(serial_number_width) << m->serial_number() << std::endl;
    }
}