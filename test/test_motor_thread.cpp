// MotorManager and MotorThread with a motor that stops answering and cannot be written to:
// the errors are reported, the other motor is not affected, and in fault tolerant mode
// the thread keeps running.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motor_thread.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
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
        return SimulatedMotor::read();
    }
    ssize_t write() override {
        if (broken) {
            errno = ENODEV;
            return -1;
        }
        writes++;
        return SimulatedMotor::write();
    }
    std::atomic<bool> silent{false};
    std::atomic<bool> broken{false};
    std::atomic<int> writes{0};
};

class TestThread : public MotorThread {
 public:
    TestThread() :
        MotorThread(1000) {}
    std::atomic<int> cycles{0};
    std::atomic<int> fresh[2] = {};
    std::atomic<int> write_failed[2] = {};
    uint32_t overruns_seen() const { return overruns(); }

 protected:
    void controller_update() override {
        cycles++;
        for (int i = 0; i < 2; i++) {
            fresh[i] += data_.status_fresh[i];
            write_failed[i] += data_.write_failed[i];
        }
        motor_manager_.set_command_mode(VELOCITY);
        motor_manager_.set_command_velocity({1, 1});
    }
};

static void test_manager() {
    auto flaky = std::make_shared<FlakyMotor>("flaky");
    auto good = std::make_shared<FlakyMotor>("good");
    MotorManager m;
    m.set_motors({flaky, good});
    assert(m.multipoll(1000 * 1000) == 2);

    // A motor that cannot be written to: the error is thrown, after the other motor has
    // got its command too.
    flaky->broken = true;
    m.set_command_mode(VELOCITY);
    bool thrown = false;
    try {
        m.write_saved_commands();
    } catch (RuntimeException& e) {
        thrown = std::string(e.what()).find("flaky") != std::string::npos;
    }
    assert(thrown);
    assert(good->writes == 1);
    assert(m.get_write_error_count()[0] == 1);
    assert(m.get_write_error_count()[1] == 0);

    // Without throwing: the same is only counted.
    m.set_throw_on_error(false);
    m.write_saved_commands();
    assert(good->writes == 2);
    assert(m.get_write_error_count()[0] == 2);
    flaky->broken = false;
    m.write_saved_commands();
    assert(m.get_write_error_count()[0] == 0);

    // A status is fresh when it is new, and not when the motor had nothing.
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
}

static void test_thread() {
    auto flaky = std::make_shared<FlakyMotor>("flaky");
    auto good = std::make_shared<FlakyMotor>("good");
    TestThread thread;
    thread.motor_manager().set_motors({flaky, good});
    thread.set_fault_tolerant();
    thread.set_scheduler(RealtimeThread::NORMAL);
    thread.set_lock_memory(false);
    thread.set_poll_timeout(100 * 1000);
    thread.init();
    assert(flaky->is_nonblocking());
    thread.run();
    assert(thread.setup_errors().empty());

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int cycles = thread.cycles;
    assert(cycles > 100);
    assert(thread.fresh[0] > 100);
    assert(thread.write_failed[0] == 0);

    // One motor goes away entirely. The thread keeps its rate and the other motor its commands.
    flaky->silent = true;
    flaky->broken = true;
    int fresh_before = thread.fresh[0];
    int good_fresh_before = thread.fresh[1];
    int good_writes_before = good->writes;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    assert(thread.cycles > cycles + 100);
    assert(thread.fresh[0] <= fresh_before + 1);
    assert(thread.fresh[1] > good_fresh_before + 100);
    assert(good->writes > good_writes_before + 100);
    assert(thread.write_failed[0] > 100);
    assert(thread.write_failed[1] == 0);

    // and comes back
    flaky->silent = false;
    flaky->broken = false;
    fresh_before = thread.fresh[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(thread.fresh[0] > fresh_before + 50);

    thread.done();
    thread.done();   // harmless when it is not running
    int stopped = thread.cycles;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    assert(thread.cycles == stopped);
    std::cout << "cycles " << stopped << " overruns " << thread.overruns_seen() << " jitter errors " << thread.jitter_errors() << std::endl;
}

int main() {
    test_manager();
    test_thread();
    return 0;
}
