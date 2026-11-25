// MotorManager::multipoll() waits until every motor has something to read, or the timeout.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motor_manager.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <time.h>
#include <unistd.h>

using namespace obot;

// A motor that has nothing to read until make_ready().
class PipeMotor : public SimulatedMotor {
 public:
    PipeMotor(std::string name) :
        SimulatedMotor(name) {
        ::close(fd_);
        assert(::pipe(pipe_) == 0);
        fd_ = pipe_[0];
    }
    ~PipeMotor() override { ::close(pipe_[1]); }
    void make_ready() { assert(::write(pipe_[1], "x", 1) == 1); }

 private:
    int pipe_[2];
};

static double thread_cpu_s() {
    struct timespec t;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static double wall_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main() {
    // All motors ready: their number, at once.
    {
        MotorManager m;
        m.set_motors({std::make_shared<SimulatedMotor>("a"), std::make_shared<SimulatedMotor>("b")});
        double t = wall_s();
        assert(m.multipoll(50 * 1000 * 1000) == 2);
        assert(wall_s() - t < 0.02);
    }

    // One motor ready and one not: the timeout, and the wait for it does not spin.
    {
        MotorManager m;
        m.set_motors({std::make_shared<SimulatedMotor>("ready"), std::make_shared<PipeMotor>("late")});
        double t = wall_s(), cpu = thread_cpu_s();
        assert(m.multipoll(50 * 1000 * 1000) == -ETIMEDOUT);
        double waited = wall_s() - t, used = thread_cpu_s() - cpu;
        std::cout << "one of two ready: waited " << waited * 1e3 << " ms, cpu " << used * 1e3 << " ms" << std::endl;
        assert(waited > 0.045 && waited < 0.2);
        assert(used < 0.01);
    }

    // The late motor answers within the timeout: all ready then.
    {
        auto late = std::make_shared<PipeMotor>("late");
        MotorManager m;
        m.set_motors({std::make_shared<SimulatedMotor>("ready"), late});
        std::thread answer([&] {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            late->make_ready();
        });
        double t = wall_s();
        assert(m.multipoll(500 * 1000 * 1000) == 2);
        double waited = wall_s() - t;
        answer.join();
        assert(waited > 0.005 && waited < 0.3);
        // and again, with nothing changed: still all ready
        assert(m.multipoll(1000 * 1000) == 2);
    }

    // A timeout of a second or more is waited for.
    {
        MotorManager m;
        m.set_motors({std::make_shared<PipeMotor>("late")});
        double t = wall_s();
        assert(m.multipoll(1050 * 1000 * 1000) == -ETIMEDOUT);
        assert(wall_s() - t > 1.0);
    }
    return 0;
}
