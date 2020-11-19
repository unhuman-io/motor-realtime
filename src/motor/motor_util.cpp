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
#include "motor_publisher.h"
#include <sstream>

struct cstr{char s[100];};
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
    bool host_time;
    bool publish;
    bool csv;
    bool reserved_float;
};

bool signal_exit = false;
int main(int argc, char** argv) {
    CLI::App app{"Utility for communicating with motor drivers"};
    bool print = false, verbose_list = false, no_list = false, version = false, list_names=false, list_path=false, list_devpath=false, list_serial_number=false;
    bool user_space_driver = false;
    std::vector<std::string> names = {};
    std::vector<std::string> paths = {};
    std::vector<std::string> devpaths = {};
    std::vector<std::string> serial_numbers = {};
    Command command = {};
    std::vector<std::pair<std::string, ModeDesired>> mode_map{
        {"open", ModeDesired::OPEN}, {"damped", ModeDesired::DAMPED}, {"current", ModeDesired::CURRENT}, 
        {"position", ModeDesired::POSITION}, {"torque", ModeDesired::TORQUE}, {"impedance", ModeDesired::IMPEDANCE}, 
        {"velocity", ModeDesired::VELOCITY}, {"current_tuning", ModeDesired::CURRENT_TUNING},
        {"position_tuning", ModeDesired::POSITION_TUNING}, {"voltage", ModeDesired::VOLTAGE}, 
        {"phase_lock", ModeDesired::PHASE_LOCK}, {"stepper_tuning", ModeDesired::STEPPER_TUNING},
        {"reset", ModeDesired::RESET}};
    std::string set_api_data;
    bool api_mode = false;
    bool run_stats = false;
    ReadOptions read_opts = { .poll = false, .aread = false, .frequency_hz = 1000, .statistics = false, .text = false , .timestamp_in_seconds = false, .host_time = false, .publish = false, .csv = false};
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
    read_option->add_flag("-t,--host-time-seconds",read_opts.host_time, "Print host read time");
    read_option->add_flag("--publish", read_opts.publish, "Publish joint data to shared memory");
    read_option->add_flag("--csv", read_opts.csv, "Convenience to set --no-list, --host-time-seconds, and --timestamp-in-seconds");
    read_option->add_flag("-f,--reserved-float", read_opts.reserved_float, "Interpret reserved 1 & 2 as floats rather than uint32");
    app.add_flag("-l,--list", verbose_list, "Verbose list connected motors");
    app.add_flag("--no-list", no_list, "Do not list connected motors");
    app.add_flag("-v,--version", version, "Print version information");
    app.add_flag("--list-names-only", list_names, "Print only connected motor names");
    app.add_flag("--list-path-only", list_path, "Print only connected motor paths");
    app.add_flag("--list-devpath-only", list_devpath, "Print only connected motor devpaths");
    app.add_flag("--list-serial-number-only", list_serial_number, "Print only connected motor serial numbers");
    app.add_flag("-u,--user-space", user_space_driver, "Connect through user space usb");
    app.add_option("-n,--names", names, "Connect only to NAME(S)")->type_name("NAME")->expected(-1);
    app.add_option("-p,--paths", paths, "Connect only to PATHS(S)")->type_name("PATH")->expected(-1);
    app.add_option("-d,--devpaths", devpaths, "Connect only to DEVPATHS(S)")->type_name("DEVPATH")->expected(-1);
    app.add_option("-s,--serial_numbers", serial_numbers, "Connect only to SERIAL_NUMBERS(S)")->type_name("SERIAL_NUMBER")->expected(-1);
    auto set_api = app.add_option("--set_api", set_api_data, "Send API data (to set parameters)");
    app.add_flag("--api", api_mode, "Enter API mode");
    app.add_flag("--run-stats", run_stats, "Check firmware run timing");
    CLI11_PARSE(app, argc, argv);

    signal(SIGINT,[](int signum){signal_exit = true;});

    if (*read_option && read_opts.csv) {
        read_opts.timestamp_in_seconds = true;
        read_opts.host_time = true;
        no_list = true;
    }

    MotorManager m(user_space_driver);
    std::vector<std::shared_ptr<Motor>> motors;
    if (names.size()) {
        motors = m.get_motors_by_name(names);
    }
    if (paths.size()) {
        auto tmp_motors = m.get_motors_by_path(paths);
        motors.insert(motors.end(), tmp_motors.begin(), tmp_motors.end());
    }
    if (devpaths.size()) {
        auto tmp_motors = m.get_motors_by_devpath(devpaths);
        motors.insert(motors.end(), tmp_motors.begin(), tmp_motors.end());
    }
    if (serial_numbers.size()) {
        auto tmp_motors = m.get_motors_by_serial_number(serial_numbers);
        motors.insert(motors.end(), tmp_motors.begin(), tmp_motors.end());
    }
    m.set_motors(motors);
    if (!names.size() && !paths.size() && !devpaths.size() && !serial_numbers.size()) {
        motors = m.get_connected_motors();
    }

    // remove null motors
    auto i = std::begin(motors);
    while (i != std::end(motors)) {
        if (!*i) {
            i = motors.erase(i);
        } else {
            ++i;
        }
    }

    if (version) {
        std::cout << "motor_util version: " << RT_VERSION_STRING << std::endl;
    }

    if (!no_list) {
        int name_width = 10;
        int serial_number_width = 15;
        int version_width = verbose_list ? 45 : 12;
        int path_width = 15;
        int dev_path_width = 12;
        if (list_names || list_path || list_devpath || list_serial_number) {
              if (motors.size() > 0) {
                    for (auto m : motors) {
                        if (list_names) {
                            std::cout << m->name();
                        } else if (list_path) {
                            std::cout << m->base_path();
                        } else if (list_devpath) {
                            std::cout << m->dev_path();
                        } else if (list_serial_number)  {
                            std::cout << m->serial_number();
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
                            << std::setw(version_width) << (verbose_list ? m->version() : m->short_version())
                            << std::setw(path_width) << m->base_path() << std::endl;
                }
            }
        }
    }

    if (run_stats && motors.size()) {
        std::cout << "name, fast_loop_cycles, fast_loop_period, main_loop_cycles, main_loop_period" << std::endl;
        for (auto m : motors) {
            auto max_api_val = [](TextAPIItem a, int num = 100) {
                int out = 0;
                for (int i=0;i<num;i++) {
                    out = std::max(std::atoi(a.get().c_str()), out);
                } 
                return out;
            };
            std::cout << m->name() << ", ";
            std::cout << max_api_val((*m)["t_exec_fastloop"]) << ", ";
            std::cout << max_api_val((*m)["t_period_fastloop"]) << ", ";
            std::cout << max_api_val((*m)["t_exec_mainloop"]) << ", ";
            std::cout << max_api_val((*m)["t_period_mainloop"]) << std::endl;
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
        std::thread t([&s,&sin]() { while(!signal_exit) { std::cin >> s; sin = true; } });
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
        pthread_cancel(t.native_handle());
        t.join();
    }

    if (*read_option) {
        if (read_opts.text) {
            while(!signal_exit) {
                char data[64];
                auto nbytes = m.motors()[0]->motor_text()->read(data,64);
                if (nbytes > 0) {
                    data[nbytes] = 0;
                    std::cout << data << std::endl;
                }
            }
        } else {
            if (read_opts.statistics) {
                std::cout << "period_avg std_dev min max read_time_avg std_dev min max";
            } else {
                if (read_opts.host_time) {
                    std::cout << "t_host,";
                }
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
            MotorPublisher<cstr> pub;
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

                if (read_opts.publish) {
                    std::ostringstream oss;
                    for (auto stat : status) {
                        oss << stat.joint_position << " ";
                    }
                    oss << std::endl;
                    cstr c;
                    std::strncpy(c.s, oss.str().c_str(), 100);
                    pub.publish(c);
                }

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
                    std::cout << std::setprecision(9);
                    if (!read_opts.reserved_float) {
                        std::cout << reserved_uint32;
                    }
                    if (read_opts.host_time) {
                        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - start_time).count()/1e9 << ",";
                    }
                    if (read_opts.timestamp_in_seconds) {
                        
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