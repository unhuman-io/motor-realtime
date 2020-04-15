#include "motor_manager.h"
#include "motor.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "rt_version.h"
#include "CLI11.hpp"
#include <queue>
#include <signal.h>
#include <string>

class Statistics {
 public:
    Statistics(int size) : size_(size) {}
    void push(double value) {
        double old_value = 0;
        if (queue_.size() >= size_) {
            old_value = queue_.front();
            queue_.pop_front();
        }
        queue_.push_back(value);
        value_sum_ += value - old_value;
        value_squared_sum_ += pow(value, 2) - pow(old_value, 2);
    }
    double get_mean() const {
        return value_sum_/queue_.size();
        //return std::accumulate(std::begin(queue_), std::end(queue_), 0)/queue_.size();
    }
    double get_stddev() const {
        if (queue_.size() > 1) {
            double variance = value_squared_sum_ - 2*value_sum_*get_mean() + pow(get_mean(),2)*queue_.size();
            return sqrt(variance/(queue_.size()-1));
        } else {
            return 0;
        }
    }
    double get_min() const {
        return *std::min_element(std::begin(queue_), std::end(queue_));
    }
    double get_max() const {
        return *std::max_element(std::begin(queue_), std::end(queue_));
    }
 private:
    int size_;
    double value_sum_ = 0, value_squared_sum_ = 0;
    std::deque<double> queue_;
};

struct ReadOptions {
    bool poll;
    bool aread;
    double frequency_hz;
    bool statistics;
    bool text;
    bool timestamp_in_seconds;
};

bool signal_exit = false;
int main(int argc, char** argv) {
    CLI::App app{"Utility for communicating with motor drivers"};
    bool print = false, list = true, version = false, list_names=false, list_path=false;
    bool user_space_driver = false;
    std::vector<std::string> names = {};
    Command command = {};
    std::vector<std::pair<std::string, ModeDesired>> mode_map{
        {"open", ModeDesired::OPEN}, {"damped", ModeDesired::DAMPED}, {"current", ModeDesired::CURRENT}, 
        {"position", ModeDesired::POSITION}, {"torque", ModeDesired::TORQUE}, {"impedance", ModeDesired::IMPEDANCE}, 
        {"velocity", ModeDesired::VELOCITY}, {"current_tuning", ModeDesired::CURRENT_TUNING},
        {"position_tuning", ModeDesired::POSITION_TUNING}, {"voltage", ModeDesired::VOLTAGE}, {"phase_lock", ModeDesired::PHASE_LOCK}, 
        {"reset", ModeDesired::RESET}};
    std::string set_api_data;
    bool api_mode = false;
    ReadOptions read_opts = { .poll = false, .aread = false, .frequency_hz = 1000, .statistics = false, .text = false , .timestamp_in_seconds = false };
    auto set = app.add_subcommand("set", "Send data to motor(s)");
    set->add_option("--host_time", command.host_timestamp, "Host time");
    set->add_option("--mode", command.mode_desired, "Mode desired")->transform(CLI::CheckedTransformer(mode_map, CLI::ignore_case));
    set->add_option("--current", command.current_desired, "Current desired");
    set->add_option("--position", command.position_desired, "Position desired");
    set->add_option("--velocity", command.velocity_desired, "Velocity desired");
    set->add_option("--torque", command.torque_desired, "Torque desired");
    set->add_option("--reserved", command.reserved, "Reserved command");
    auto read_option = app.add_subcommand("read", "Print data received from motor(s)");
    read_option->add_flag("-s,--timestamp-in-seconds", read_opts.timestamp_in_seconds, "Report motor timestamp as seconds since start and unwrap");
    read_option->add_flag("--poll", read_opts.poll, "Use poll before read");
    read_option->add_flag("--aread", read_opts.aread, "Use aread before poll");
    read_option->add_option("--frequency", read_opts.frequency_hz , "Read frequency in Hz");
    read_option->add_flag("--statistics", read_opts.statistics, "Print statistics rather than values");
    read_option->add_flag("--text",read_opts.text, "Read the text interface instead");
    app.add_flag("-l,--list,!--no-list", list, "List connected motors");
    app.add_flag("-v,--version", version, "Print version information");
    app.add_flag("--list-names-only", list_names, "Print only connected motor names");
    app.add_flag("--list-path-only", list_path, "Print only connected motor paths");
    app.add_flag("-u,--user-space", user_space_driver, "Connect through user space usb");
    app.add_option("-n,--names", names, "Connect only to NAME(S)")->type_name("NAME")->expected(-1);
    auto set_api = app.add_option("--set_api", set_api_data, "Send API data (to set parameters)");
    app.add_flag("--api", api_mode, "Enter API mode");
    CLI11_PARSE(app, argc, argv);

    MotorManager m;
    std::vector<std::shared_ptr<Motor>> motors;
    if (names.size()) {
        motors = m.get_motors_by_name(names, user_space_driver);
        auto i = std::begin(motors);
        while (i != std::end(motors)) {
            if (!*i) {
                i = motors.erase(i);
            } else {
                ++i;
            }
        }
    } else {
        motors = m.get_connected_motors(user_space_driver);
    }

    if (version) {
        std::cout << "motor_util version: " << RT_VERSION_STRING << std::endl;
    }

    if (list) {
        int name_width = 10;
        int serial_number_width = 15;
        int version_width = 60;
        int path_width = 15;
        int dev_path_width = 12;
        if (list_names || list_path) {
              if (motors.size() > 0) {
                    for (auto m : motors) {
                        if (list_names) {
                            std::cout << m->name();
                        } else if (list_path) {
                            std::cout << m->base_path();
                        }
                        std::cout << std::endl;
                    }
              }
        } else {
            std::cout << motors.size() << " connected motor" << (motors.size() == 1 ? "" : "s") << std::endl;
            if (motors.size() > 0) {
                std::cout << std::setw(dev_path_width) << "Dev" << std::setw(name_width) << "Name"
                            << std::setw(serial_number_width) << " Serial number"
                            << std::setw(version_width) << "Version" << std::setw(path_width) << "Path" << std::endl;
                std::cout << std::setw(dev_path_width + name_width + serial_number_width + version_width + path_width) << std::setfill('-') << "" << std::setfill(' ') << std::endl;
                for (auto m : motors) {
                    std::cout << std::setw(dev_path_width) << m->dev_path()
                            << std::setw(name_width) << m->name()
                            << std::setw(serial_number_width) << m->serial_number()
                            << std::setw(version_width) << m->version()
                            << std::setw(path_width) << m->base_path() << std::endl;
                }
            }
        }
    }

    if (*set && motors.size()) {
        auto commands = std::vector<Command>(motors.size(), command);
        std::cout << "Writing commands: \n" << m.command_headers() << std::endl << commands << std::endl;
        m.write(commands);
    }

    if (*set_api || api_mode || *read_option && read_opts.text) {
        if (motors.size() != 1) {
            std::cout << "Select one motor to use api mode" << std::endl;
            return 1;
        }
    }

    if (*set_api && motors.size()) {
        auto nbytes = m.motors()[0]->motor_text()->write(set_api_data.c_str(), set_api_data.size());
        std::cout << "wrote " << nbytes << " bytes: " << set_api_data << std::endl;
    }

    if (api_mode) {
        std::string s;
        bool sin = false;
        std::thread t([&s,&sin]() { while(1) { std::cin >> s; sin = true; } });
        while(!signal_exit) {
            char data[64];
            auto nbytes = m.motors()[0]->motor_text()->read(data,64);
            if (nbytes > 0) {
                data[nbytes] = 0;
                std::cout << data << std::endl;
            }
            if (sin) {
               // std::cout << "s: " << s << "n: " << s.size() << std::endl;
                m.motors()[0]->motor_text()->write(s.c_str(), s.size());
                sin = false;
            }
        }
        t.join();
    }

    if (*read_option) {
        if (read_opts.text) {
            signal(SIGINT,[](int signum){signal_exit = true;});
            while(!signal_exit) {
                char data[64];
                auto nbytes = m.motors()[0]->motor_text()->read(data,64);
                if (nbytes > 0) {
                    data[nbytes] = 0;
                    std::cout << data << std::endl;
                }
            }
        } else {
            signal(SIGINT,[](int signum){signal_exit = true;});
            if (read_opts.statistics) {
                ; //todo
            } else {
                if (read_opts.timestamp_in_seconds) {
                    int length = motors.size();
                    for (int i=0;i<length;i++) {
                        std::cout << "t_seconds" << i << ", ";
                    }
                }
                std::cout << m.status_headers() << std::endl;
            }
            auto start_time = std::chrono::steady_clock::now();
            auto next_time = start_time;
            auto loop_start_time = start_time;
            int64_t period_ns = 1e9/read_opts.frequency_hz;
            Statistics exec(100), period(100);
            int i = 0;
            while (!signal_exit) {
                auto last_loop_start_time = loop_start_time;
                loop_start_time = std::chrono::steady_clock::now();
                next_time += std::chrono::nanoseconds(period_ns);
                if (read_opts.aread) {
                    m.aread();
                }
                if (read_opts.poll) {
                    m.poll();
                }
                auto status = m.read();
                auto exec_time = std::chrono::steady_clock::now();

                if (read_opts.statistics) {
                    i++;
                    auto last_exec = std::chrono::duration_cast<std::chrono::nanoseconds>(exec_time - loop_start_time).count();
                    auto last_start = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - start_time).count();
                    auto last_period = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - last_loop_start_time).count();
                    exec.push(last_exec);
                    period.push(last_period);
                    if (i > 100) {
                        i = 0;
                        auto width = 15;
                        std::cout << std::fixed << std::setprecision(0) << std::setw(width) << last_start << std::setw(width) << floor(period.get_mean()) << std::setw(width) << 
                        period.get_stddev() << std::setw(width) << period.get_min()  << std::setw(width) << period.get_max() << std::setw(width) <<
                        floor(exec.get_mean()) << std::setw(width) <<  exec.get_stddev() << std::setw(width) << exec.get_min() << std::setw(width) << exec.get_max()  << std::endl;
                    }
                } else {
                    std::cout << std::fixed;
                    if (read_opts.timestamp_in_seconds) {
                        std::cout << std::setprecision(9);
                        static auto last_status = status;
                        static double *t_seconds = new double[status.size()]();
                        for (int i = 0; i < status.size(); i++) {
                            uint32_t dt = status[i].mcu_timestamp - last_status[i].mcu_timestamp;
                            t_seconds[i] += dt/170.0e6;
                            std::cout << t_seconds[i] << ", ";
                        }
                        last_status = status;
                    }
                    std::cout << std::setprecision(5);
                    std::cout << status << std::endl;
                }

                // option to not sleep
                // while (std::chrono::steady_clock::now() < next_time);
                std::this_thread::sleep_until(next_time);
            }
        }
    }

    return 0;
}