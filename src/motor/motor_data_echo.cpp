#include "motor_subscriber.h"
#include <thread>
#include <iostream>

int main(int argc, char** argv) {
    MotorSubscriber sub;
    while(1) {
        std::string str = sub.read();
        if (str.size()) {
            std::cout << sub.read();
        } else {
            std::cout << "no data" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}