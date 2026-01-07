#include "motor.h"
#include "l2.h"
#include "CLI11.hpp"
#include "realtime_thread.h"
#include <cxxabi.h>

namespace obot {

class MotorL2Chain {
  public:
    MotorL2Chain(std::string interface, std::string mac_address, std::vector<std::string> names) :
        mac_address_(mac_address), socket_(interface, mac_address) {
        std::cout << "Simulating motors:";
        for (const auto &name : names) {
            motors_.push_back(std::make_unique<SimulatedMotor>(name));
            std::cout << " " << name;
        }
        std::cout << std::endl;
        std::cout << "interface: " << interface << std::endl;
        std::cout << "MAC address: " << mac_address_ << std::endl;
    }
    void update() {
        std::cout << ".";

        std::cout << socket_.recv();

        uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE];
        int ptr = 0;
        struct {
            uint16_t node_id:4 = 1;
            uint16_t bus_id:4 = 0;
            uint16_t type:4 = 2; // status
        } type;
        for (auto &motor : motors_) {
            auto length = motor->read();
            if (ptr + 3 + length >= sizeof(payload)) { break; }
            std::memcpy(&payload[ptr], &type, 2);
            ptr += 2;
            type.node_id++;
            payload[ptr++] = length;
            std::memcpy(&payload[ptr], motor->status(), length);
            ptr += length;
        }
        socket_.send((const char *) payload, ptr);
    }
  private:
    std::vector<std::unique_ptr<SimulatedMotor>> motors_;
    std::string mac_address_;
    L2Device socket_;
};

} // namespace obot

using namespace obot;

int _main(int argc, char** argv) {
    std::vector<std::string> names {"sim1"};
    std::string mac_address {"00:00:00:00:00:00"};
    std::string interface {"lo"};
    CLI::App app{"Utility for simulating motor drivers with l2 socket communication"};
    app.add_option("-n,--names", names, "Create NAME(S) simulated motors")->type_name("NAME")->capture_default_str()->expected(1,-1);
    app.add_option("-m,--mac", mac_address, "Use MAC address MAC_ADDRESS")->type_name("MAC_ADDRESS")->capture_default_str()->expected(1);
    app.add_option("-i,--interface", interface, "Use network interface INTERFACE")->type_name("INTERFACE")->capture_default_str()->expected(1);
    CLI11_PARSE(app, argc, argv);
    MotorL2Chain chain(interface, mac_address, names);
    RealtimeThread thread(1000, [&chain](){chain.update();});
    thread.run();
    while(1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << '*';
        std::cout.flush();
    }
    return 0;
}

int main(int argc, char** argv) {
    try {
        return _main(argc, argv);
    } catch (const RuntimeException &e) {
        std::cerr << "Caught RuntimeException" << std::endl;
        std::cerr << " what(): " << e.what() << std::endl;
        std::cerr << e.location_print() << std::endl;    
    } catch (const std::exception &e) {
        int status;
        std::cerr << "Caught exception of type " << abi::__cxa_demangle(typeid(e).name(), NULL, NULL, &status) << std::endl;
        std::cerr << "  what():  " << e.what() << std::endl;
        return 1;
    }
}