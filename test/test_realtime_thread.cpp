// RealtimeThread: starting and stopping, and the schedulers other than the default.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "realtime_thread.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace obot;

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    // done() on a thread that never ran does nothing.
    {
        RealtimeThread thread(1000);
        thread.done();
    }

    // It runs at its rate, stops, can be stopped again and started again.
    {
        std::atomic<int> cycles{0};
        RealtimeThread thread(1000, [&] { cycles++; });
        thread.run();
        sleep_ms(100);
        thread.done();
        int stopped = cycles;
        std::cout << stopped << " cycles in 100 ms at 1000 Hz" << std::endl;
        assert(stopped > 50 && stopped < 150);
        thread.done();
        sleep_ms(10);
        assert(cycles == stopped);

        thread.run();
        sleep_ms(50);
        thread.done();
        assert(cycles > stopped + 20);

        // stopped right after it was started, many times
        for (int i = 0; i < 200; i++) {
            thread.run();
            thread.done();
        }
    }

    // An update that takes longer than done() waits (20 periods): done() gives up on it,
    // says so, and the program goes on.
    {
        std::atomic<bool> block{false}, finished{false};
        RealtimeThread thread(1000, [&] {
            if (block) {
                sleep_ms(100);
                finished = true;
            }
        });
        thread.run();
        sleep_ms(10);
        block = true;
        sleep_ms(10);
        thread.done();
        assert(!finished);
        // (the thread object has to outlive its update)
        sleep_ms(150);
        assert(finished);
    }

    // NORMAL: plain sleeping until each period, nothing to set up that could fail.
    {
        std::atomic<int> cycles{0};
        RealtimeThread thread(1000, [&] { cycles++; });
        thread.set_scheduler(RealtimeThread::NORMAL);
        thread.set_lock_memory(false);
        assert(thread.setup_errors().empty());
        thread.run();
        // run() returns when the thread has set itself up
        assert(thread.setup_errors().empty());
        sleep_ms(100);
        thread.done();
        assert(cycles > 50 && cycles < 120);
        assert(thread.overruns() == 0);

        // a period that is not a whole number of Hz
        thread.set_period_ns(400 * 1000);
        assert(thread.get_period_ns() == 400 * 1000);
        cycles = 0;
        thread.run();
        sleep_ms(100);
        thread.done();
        std::cout << cycles << " cycles in 100 ms with a period of 400 us" << std::endl;
        assert(cycles > 120 && cycles < 300);
    }

    // A cycle that runs past its period: the periods it missed are skipped and counted,
    // not caught up with a burst of cycles.
    {
        std::atomic<int> cycles{0}, after_slow{-1};
        std::atomic<bool> slow{false};
        std::chrono::steady_clock::time_point starts[4];
        RealtimeThread thread(1000, [&] {
            cycles++;
            if (after_slow >= 0 && after_slow < 4) {
                starts[after_slow++] = std::chrono::steady_clock::now();
            }
            if (slow.exchange(false)) {
                std::this_thread::sleep_for(std::chrono::microseconds(3500));
                after_slow = 0;
            }
        });
        thread.set_scheduler(RealtimeThread::NORMAL);
        thread.set_lock_memory(false);
        thread.run();
        sleep_ms(20);
        slow = true;
        sleep_ms(100);
        thread.done();
        int spaced = 0;
        for (int i = 1; i < 4; i++) {
            spaced += starts[i] - starts[i - 1] > std::chrono::microseconds(500);
        }
        std::cout << cycles << " cycles in 120 ms with one cycle of 3.5 ms, overruns " << thread.overruns()
                  << ", " << spaced << " of the 3 cycles after it a period apart" << std::endl;
        assert(after_slow == 4);
        assert(thread.overruns() >= 3);
        assert(spaced >= 2);
    }

    // FIFO on a CPU: either it is permitted, or setup_errors() says what was not and the
    // thread runs all the same.
    {
        std::atomic<int> cycles{0};
        RealtimeThread thread(1000, [&] { cycles++; });
        thread.set_scheduler(RealtimeThread::FIFO, 10);
        thread.set_cpu(0);
        thread.set_lock_memory(false);
        thread.run();
        for (auto& e : thread.setup_errors()) {
            std::cout << "FIFO: " << e << std::endl;
        }
        sleep_ms(50);
        thread.done();
        assert(cycles > 30);
    }

    // The default is as before: SCHED_DEADLINE, or sleeping where that is not permitted.
    {
        std::atomic<int> cycles{0};
        RealtimeThread thread(1000, [&] { cycles++; });
        thread.run();
        for (auto& e : thread.setup_errors()) {
            std::cout << "default: " << e << std::endl;
        }
        sleep_ms(50);
        thread.done();
        assert(cycles > 30);
    }
    return 0;
}
