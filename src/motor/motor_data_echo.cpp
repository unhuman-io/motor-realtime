#include "motor_subscriber.h"
#include <thread>
#include <iostream>

struct cstr{char s[100];};

int main(int /* argc */, char** /* argv */) {
    MotorSubscriber<cstr> sub;
    while(1) {
        std::string str = sub.read().s;
        if (str.size()) {
            std::cout << str;
        } else {
            std::cout << "no data" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}
