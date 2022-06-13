#include <motor_manager.h>
#include <string>

int main (int /* argc */, char ** /* argv */) {
    MotorManager m;
    m.get_connected_motors();
    double kp;
    kp << (*m.motors()[0])["kp"];
    std::cout << "kp = " << kp << std::endl;
    (*m.motors()[0])["kp"] = std::to_string(kp + 1);
    std::cout << "kp = " << (*m.motors()[0])["kp"] << std::endl;
    return 0;
}
