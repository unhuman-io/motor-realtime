#ifdef NDEBUG
#undef NDEBUG
#endif
#include "can_time_sync.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

using namespace obot;

// in ns, as the simulation counts
static int64_t host_time_ns(CANDeviceClock &clock, int64_t device_ns, int64_t kernel_ns) {
    return clock.host_time(std::chrono::nanoseconds(device_ns), CANTime(std::chrono::nanoseconds(kernel_ns)))
        .time_since_epoch().count();
}

// CANDeviceClock against an interface that is simulated: a clock of its own
// that started at some time and runs fast or slow, and a kernel time for each
// frame that is late by an interrupt, which takes 20 us at least, 30 us more
// on average, and a millisecond now and then.
struct Simulation {
    double period_s, rate;
    int frames;
    double least_late_us = 1e9, most_late_us = -1e9;    // the host time, past the warm up

    void run(int64_t jump_at_frame = -1) {
        std::mt19937_64 random(static_cast<uint64_t>(period_s * 1e6) + frames);
        std::exponential_distribution<double> interrupt_ns(1 / 30e3);
        CANDeviceClock clock;
        const int64_t start_ns = 1794950000ll * 1000000000;
        int64_t device_start_ns = 5000ll * 1000000000;     // 5000 s on the interface's clock
        int since_jump = 0;
        for (int i = 0; i < frames; i++) {
            const int64_t elapsed_ns = static_cast<int64_t>(i * period_s * 1e9);
            if (i == jump_at_frame) {      // the interface's clock starts over
                device_start_ns = -elapsed_ns;
                since_jump = 0;
            }
            const int64_t true_ns = start_ns + elapsed_ns;
            const int64_t device_ns = device_start_ns + static_cast<int64_t>(static_cast<double>(elapsed_ns) * (1 + rate));
            const int64_t late_ns = 20000 + static_cast<int64_t>(interrupt_ns(random)) + (random() % 100 == 0 ? 1000000 : 0);
            const double error_us = static_cast<double>(host_time_ns(clock, device_ns, true_ns + late_ns) - true_ns) / 1e3;
            if (since_jump++ * period_s >= 40) {
                least_late_us = std::min(least_late_us, error_us);
                most_late_us = std::max(most_late_us, error_us);
            }
        }
    }
};

int main() {
    // What the host time is off by, 40 s on: late by the 20 us that an
    // interrupt takes at least, which nothing here can tell, give or take
    for (const double period_s : {1.0, 0.1, 0.001}) {
        for (const double rate : {0.0, 50e-6, -50e-6, 5e-6}) {
            Simulation simulation{period_s, rate, static_cast<int>(300 / period_s)};
            simulation.run();
            std::cout << "period " << period_s << " s, rate " << rate * 1e6 << " ppm: " << simulation.least_late_us
                      << " to " << simulation.most_late_us << " us late" << std::endl;
            assert(simulation.least_late_us > 10 && simulation.most_late_us < 40);
        }
    }

    // The interface's clock starts over: right again 40 s on
    Simulation jump{1.0, 20e-6, 400};
    jump.run(200);
    std::cout << "with the interface's clock set: " << jump.least_late_us << " to " << jump.most_late_us << " us late" << std::endl;
    assert(jump.least_late_us > 10 && jump.most_late_us < 40);

    // Times that the driver has made this host's already are taken as they are
    CANDeviceClock host_clock;
    const int64_t now_ns = 1794950000ll * 1000000000;
    assert(host_time_ns(host_clock, now_ns, now_ns + 35000) == now_ns);
    assert(host_time_ns(host_clock, now_ns + 1000000000, now_ns + 1000000000 + 900000) == now_ns + 1000000000);
    std::cout << "ok" << std::endl;
    return 0;
}
