// MotorManager with a motor that cannot be written to or does not answer: what the others
// get and what is reported, by exception or, with set_throw_on_error(false), by counts.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motor_manager.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace obot;

class FlakyMotor : public SimulatedMotor {
 public:
    FlakyMotor(std::string name) :
        SimulatedMotor(name) {}
    ssize_t read() override {
        if (silent) {
            errno = EAGAIN;
            return -1;
        }
        if (throws) {
            throw RuntimeException("Error reading " + name());
        }
        return SimulatedMotor::read();
    }
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
    std::atomic<bool> silent{false};
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

static void test_no_throw() {
    auto flaky = std::make_shared<FlakyMotor>("flaky");
    auto good = std::make_shared<FlakyMotor>("good");
    MotorManager m;
    m.set_motors({flaky, good});
    m.set_command_mode(VELOCITY);
    m.set_throw_on_error(false);

    // Writes that fail, by return value or by exception, are only counted.
    flaky->broken = true;
    m.write_saved_commands();
    assert(good->writes == 1);
    assert(m.get_write_error_count()[0] == 1);
    flaky->broken = false;
    flaky->throws = true;
    m.write_saved_commands();
    assert(good->writes == 2);
    assert(m.get_write_error_count()[0] == 2);
    assert(m.get_write_error_count()[1] == 0);

    // So are reads; the status of that motor is not fresh, the other's is.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m.read();
    assert(m.get_read_error_count()[0] == 1);
    assert(m.get_status_fresh()[0] == 0);
    assert(m.get_status_fresh()[1] == 1);
    flaky->throws = false;
    m.write_saved_commands();
    assert(m.get_write_error_count()[0] == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m.read();
    assert(m.get_read_error_count()[0] == 0);
    assert(m.get_status_fresh()[0] == 1);

    // With throwing on again a read error is an exception as before.
    m.set_throw_on_error(true);
    flaky->throws = true;
    bool thrown = false;
    try {
        m.read();
    } catch (RuntimeException& e) {
        thrown = std::string(e.what()) == "Error reading flaky";
    }
    assert(thrown);
}

// A status is fresh when it is new, and not when a nonblocking motor had nothing.
static void test_fresh() {
    auto flaky = std::make_shared<FlakyMotor>("flaky");
    auto good = std::make_shared<FlakyMotor>("good");
    MotorManager m;
    m.set_motors({flaky, good});
    flaky->set_nonblock();
    good->set_nonblock();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m.read();
    assert(m.get_status_fresh()[0] == 1);
    assert(m.get_status_fresh()[1] == 1);
    flaky->silent = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m.read();
    assert(m.get_status_fresh()[0] == 0);
    assert(m.get_status_fresh()[1] == 1);
    assert(m.get_nonblock_not_ready_error_count()[0] == 1);
    flaky->silent = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m.read();
    assert(m.get_status_fresh()[0] == 1);
}

int main() {
    test_write();
    test_no_throw();
    test_fresh();
    return 0;
}
