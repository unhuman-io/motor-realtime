#include "motor_manager.h"
#include <cstdint>
#include <chrono>
#include <thread>

using namespace obot;

int main (int /* argc */, char ** /* argv */) {
    uint32_t period_ns = 100 * 1000 * 1000;

    auto m = MotorManager();
    m.get_connected_motors();
    char data[m.serialize_command_size()];
    auto pipe_fd = open("/tmp/deadline", O_WRONLY);

    m.set_command_mode(ModeDesired::POSITION);

    uint32_t x = 0;
    auto next_time = std::chrono::steady_clock::now();
    while (1) {
        x++;
        next_time += std::chrono::nanoseconds(period_ns);

        m.set_command_count(x);
        m.set_command_position(std::vector<float>(m.motors().size(), x));
        m.set_command_velocity(std::vector<float>(m.motors().size(), .1));
        auto s = m.serialize_saved_commands(data);
        write(pipe_fd, data, s);
        printf("%d\n", x);

        std::this_thread::sleep_until(next_time);
    }
}
