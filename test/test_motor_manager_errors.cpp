// MotorManager with a motor that cannot be written to: what the others get and what is
// reported.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motor_manager.h"
#include <atomic>
#include <cassert>
#include <iostream>
#include <string>

using namespace obot;

class FlakyMotor : public SimulatedMotor {
 public:
    FlakyMotor(std::string name) :
        SimulatedMotor(name) {}
    ssize_t write() override {
        if (broken) {
            errno = ENODEV;
            return -1;
        }
        if (throws) {
            throw RuntimeException("Error writing can " + name());
        }
        writes++;
        return SimulatedMotor::write();
    }
    std::atomic<bool> broken{false};
    std::atomic<bool> throws{false};
    std::atomic<int> writes{0};
};

static std::string write_error(MotorManager& m) {
    try {
        m.write_saved_commands();
    } catch (RuntimeException& e) {
        return e.what();
    }
    return "";
}

static void test_write() {
    auto flaky = std::make_shared<FlakyMotor>("flaky");
    auto good = std::make_shared<FlakyMotor>("good");
    MotorManager m;
    m.set_motors({flaky, good});
    m.set_command_mode(VELOCITY);
    assert(write_error(m) == "");
    assert(good->writes == 1);
    assert(m.get_write_error_count()[0] == 0);

    // A motor that cannot be written to: the error is thrown, after the other motor has
    // got its command too.
    flaky->broken = true;
    assert(write_error(m) == "Error writing: flaky error 19: No such device");
    assert(good->writes == 2);
    assert(m.get_write_error_count()[0] == 1);
    assert(m.get_write_error_count()[1] == 0);
    assert(write_error(m) != "");
    assert(good->writes == 3);
    assert(m.get_write_error_count()[0] == 2);

    // A motor whose write() throws (as on CAN): the same, and its own exception arrives.
    flaky->broken = false;
    flaky->throws = true;
    assert(write_error(m) == "Error writing can flaky");
    assert(good->writes == 4);
    assert(m.get_write_error_count()[0] == 3);

    // Both fail: both are named.
    flaky->throws = false;
    flaky->broken = true;
    good->broken = true;
    assert(write_error(m) == "Error writing: flaky error 19: No such device\nError writing: good error 19: No such device");
    assert(m.get_write_error_count()[1] == 1);

    // Working again: the counts start over.
    flaky->broken = false;
    good->broken = false;
    assert(write_error(m) == "");
    assert(m.get_write_error_count()[0] == 0);
    assert(m.get_write_error_count()[1] == 0);
}

int main() {
    test_write();
    return 0;
}
