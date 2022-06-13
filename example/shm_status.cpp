#include <motor_manager.h>
#include <thread>
#include <motor_subscriber.h>

int main() {
    MotorSubscriber<MotorStatus> sub;

    while(1) {
        MotorStatus status = sub.read();
        std::vector<MotorStatus> statuses(1, status);
        std::cout << statuses << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}
