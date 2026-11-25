// RealtimeThread: starting and stopping.
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
    return 0;
}
