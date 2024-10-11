#include "motor_manager.h"
#include "motor.h"
#include "motor_uart.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "rt_version.h"
#include "CLI11.hpp"
#include <queue>
#include <signal.h>
#include <string>
#include <sstream>
#include "realtime_thread.h"
#include "keyboard.h"
#include "motor_util_fun.h"
#include <json.hpp>

using namespace obot;

struct cstr{char s[100];};
class Statistics {
 public:
    Statistics(int size = 100) : size_(size) {}
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
    bool ppoll;
    bool aread;
    bool nonblock;
    double frequency_hz;
    bool statistics;
    std::vector<std::string> text;
    bool timestamp_in_seconds;
    bool host_time;
    bool csv;
    bool reconnect;
    bool read_write_statistics;
    std::vector<double> bits;
    bool compute_velocity;
    double timestamp_frequency_hz;
    int precision;
    bool mini;
    bool fastlog;
};

bool signal_exit = false;
int main(int argc, char** argv) {
    CLI::App app{"Utility for communicating with motor drivers\n"
                 "\n"
                 "Use the environment variable MOTOR_UTIL_CONFIG_DIR to set the configuration directory\n"
                 "Or else the configuration path is checked in order:\n"
                 "    ~/.config/motor_util/\n"
                 "    /etc/motor_util/\n"
                 "    /usr/share/motor-realtime/\n"};
    bool verbose_list = false, no_list = false, version = false, list_names=false, list_path=false, list_devpath=false, list_serial_number=false, list_devnum=false;
    bool no_dfu_list = false;
    bool user_space_driver = false;
    std::vector<std::string> names = {};
    std::vector<std::string> paths = {};
    std::vector<std::string> devpaths = {};
    std::vector<std::string> serial_numbers = {};
    std::vector<std::string> uart_paths = {};
    std::vector<std::string> can_devs = {"any"};
    bool uart_raw = false;
    std::vector<std::string> ips = {};
    
    std::string config_dir;
    char * config_dir_env = getenv("MOTOR_UTIL_CONFIG_DIR");
    if (config_dir_env == NULL) {
        // right now the only thing in the config directory is the device_ip_map.json
        // will have to figure out the search path implementation later if other files are added
        config_dir = std::string(getenv("HOME")) + "/.config/motor_util/";
        if (access((config_dir + "device_ip_map.json").c_str(), F_OK) != 0) {
            config_dir = "/etc/motor_util/";
            if (access((config_dir + "device_ip_map.json").c_str(), F_OK) != 0) {
                config_dir = "/usr/share/motor-realtime/";
            }
        }
    } else {
        config_dir = std::string(config_dir_env);
        if (config_dir.back() != '/') {
            config_dir += "/";
        }
    }
    std::string json_ip_file_default = config_dir + "device_ip_map.json";
    std::string json_ip_file = json_ip_file_default;
    bool no_print_unconnected = false;
    Command command = {};
    std::vector<std::pair<std::string, ModeDesired>> mode_map;
    for (const std::pair<const ModeDesired, const std::string> &pair : MotorManager::mode_map) {
        mode_map.push_back({pair.second, pair.first});
    }
    std::vector<std::pair<std::string, ModeDesired>> tuning_mode_options_map{
        {"position", ModeDesired::POSITION}, {"velocity", ModeDesired::VELOCITY}, {"torque", ModeDesired::TORQUE}
    };    
    std::vector<std::pair<std::string, TuningMode>> tuning_mode_map{
        {"sine", TuningMode::SINE}, {"square", TuningMode::SQUARE}, {"triangle", TuningMode::TRIANGLE}, 
        {"chirp", TuningMode::CHIRP}};
    std::vector<std::pair<std::string, StepperMode>> stepper_mode_map{
        {"current", StepperMode::STEPPER_CURRENT}, {"voltage", StepperMode::STEPPER_VOLTAGE}
    };
    std::vector<std::string> set_api_data;
    bool api_mode = false;
    bool api_timing = false;
    int run_stats = 100;
    int timeout_ms = 10;
    bool allow_simulated = false;
    Motor::MessagesCheck check_messages_version = Motor::MessagesCheck::MAJOR;
    std::vector<std::pair<std::string, Motor::MessagesCheck>> messages_check_map {
        {"none", Motor::MessagesCheck::NONE}, {"major", Motor::MessagesCheck::MAJOR},
        {"minor", Motor::MessagesCheck::MINOR}};
    bool command_gpio = false;
    bool lock_motors = false;
    ReadOptions read_opts = { .poll = false, .ppoll = false, .aread = false, .nonblock = false, .frequency_hz = 1000, 
        .statistics = false, .text = {"log"} , .timestamp_in_seconds = false, .host_time = false, 
        .csv = false, .reconnect = false, .read_write_statistics = false,
        .bits={100,1}, .compute_velocity = false, .timestamp_frequency_hz=170e6, .precision=5};
    auto set = app.add_subcommand("set", "Send data to motor(s)");
    set->add_option("--host_time", command.host_timestamp, "Host time");
    set->add_option("--mode", command.mode_desired, "Mode desired")->transform(CLI::CheckedTransformer(mode_map, CLI::ignore_case));
    set->add_option("--current", command.current_desired, "Current desired");
    set->add_option("--position", command.position_desired, "Position desired");
    set->add_option("--velocity", command.velocity_desired, "Velocity desired");
    set->add_option("--torque", command.torque_desired, "Torque desired");
    set->add_option("--torque_dot", command.torque_dot_desired, "Torque dot desired");
    set->add_option("--reserved", command.reserved, "Reserved command");
    set->add_option("--gpio", command_gpio, "GPIO output");
    auto state_mode = set->add_subcommand("state", "State control mode")->final_callback([&](){command.mode_desired = ModeDesired::STATE;})->fallthrough();
    state_mode->add_option("--kp", command.state.kp, "Position error gain");
    state_mode->add_option("--kd", command.state.kd, "Velocity error gain");
    state_mode->add_option("--kt", command.state.kt, "Torque error gain");
    state_mode->add_option("--ks", command.state.ks, "Torque dot error gain");
    auto stepper_tuning_mode = set->add_subcommand("stepper_tuning", "Stepper tuning mode")->final_callback([&](){command.mode_desired = ModeDesired::STEPPER_TUNING;});
    stepper_tuning_mode->add_option("--amplitude", command.stepper_tuning.amplitude, "Phase position tuning amplitude");
    stepper_tuning_mode->add_option("--frequency", command.stepper_tuning.frequency, "Phase tuning frequency hz, or hz/s for chirp");
    stepper_tuning_mode->add_option("--mode", command.stepper_tuning.mode, "Phase tuning mode")->transform(CLI::CheckedTransformer(tuning_mode_map, CLI::ignore_case));
    stepper_tuning_mode->add_option("--kv", command.stepper_tuning.kv, "Motor kv (rad/s)");
    stepper_tuning_mode->add_option("--stepper_mode", command.stepper_tuning.stepper_mode, "Current/voltage mode")->transform(CLI::CheckedTransformer(stepper_mode_map, CLI::ignore_case));
    auto position_tuning_mode = set->add_subcommand("position_tuning", "Position tuning mode")->final_callback([&](){command.mode_desired = ModeDesired::POSITION_TUNING;});
    position_tuning_mode->add_option("--amplitude", command.position_tuning.amplitude, "Position tuning amplitude");
    position_tuning_mode->add_option("--frequency", command.position_tuning.frequency, "Position tuning frequency hz, or hz/s for chirp");
    position_tuning_mode->add_option("--mode", command.position_tuning.mode, "Position tuning mode")->transform(CLI::CheckedTransformer(tuning_mode_map, CLI::ignore_case));
    position_tuning_mode->add_option("--bias", command.position_tuning.bias, "Position trajectory offset");
    auto current_tuning_mode = set->add_subcommand("current_tuning", "Current tuning mode")->final_callback([&](){command.mode_desired = ModeDesired::CURRENT_TUNING;});
    current_tuning_mode->add_option("--amplitude", command.current_tuning.amplitude, "Current tuning amplitude");
    current_tuning_mode->add_option("--frequency", command.current_tuning.frequency, "Current tuning frequency hz, or hz/s for chirp");
    current_tuning_mode->add_option("--mode", command.current_tuning.mode, "Current tuning mode")->transform(CLI::CheckedTransformer(tuning_mode_map, CLI::ignore_case));
    current_tuning_mode->add_option("--bias", command.current_tuning.bias, "Current trajectory offset");
    auto stepper_velocity_mode = set->add_subcommand("stepper_velocity", "Stepper velocity mode")->final_callback([&](){command.mode_desired = ModeDesired::STEPPER_VELOCITY;});
    stepper_velocity_mode->add_option("--voltage", command.stepper_velocity.voltage, "Phase voltage amplitude");
    stepper_velocity_mode->add_option("--velocity", command.stepper_velocity.velocity, "Phase velocity");
    stepper_velocity_mode->add_option("--current", command.stepper_velocity.current, "Current desired");
    stepper_velocity_mode->add_option("--stepper_mode", command.stepper_velocity.stepper_mode, "Current/voltage mode")->transform(CLI::CheckedTransformer(stepper_mode_map, CLI::ignore_case));
    auto tuning_mode = set->add_subcommand("tuning", "Tuning mode")->final_callback([&](){command.mode_desired = ModeDesired::TUNING;});
    tuning_mode->add_option("--amplitude", command.tuning_command.amplitude, "Tuning amplitude");
    tuning_mode->add_option("--frequency", command.tuning_command.frequency, "Tuning frequency hz, or hz/s for chirp");
    tuning_mode->add_option("--tuning_mode", command.tuning_command.tuning_mode, "Tuning mode")->transform(CLI::CheckedTransformer(tuning_mode_map, CLI::ignore_case));
    tuning_mode->add_option("--mode", command.tuning_command.mode, "Main Mode")->transform(CLI::CheckedTransformer(tuning_mode_options_map, CLI::ignore_case));
    tuning_mode->add_option("--bias", command.tuning_command.bias, "Trajectory offset");
    auto voltage_mode = set->add_subcommand("voltage", "Voltage mode")->final_callback([&](){command.mode_desired = ModeDesired::VOLTAGE;});
    voltage_mode->add_option("--voltage", command.voltage.voltage_desired, "Vq voltage desired");
    auto read_option = app.add_subcommand("read", "Print data received from motor(s)");
    read_option->add_flag("-s,--timestamp-in-seconds", read_opts.timestamp_in_seconds, "Report motor timestamp as seconds since start and unwrap");
    read_option->add_flag("--poll", read_opts.poll, "Use poll before read");
    read_option->add_flag("--ppoll", read_opts.ppoll, "Use multipoll before read");
    read_option->add_flag("--aread", read_opts.aread, "Use aread before poll");
    read_option->add_flag("--nonblock", read_opts.nonblock, "Use non-blocking i/o for read");
    read_option->add_option("--frequency", read_opts.frequency_hz , "Read frequency in Hz");
    read_option->add_flag("--statistics", read_opts.statistics, "Print statistics rather than values");
    read_option->add_flag("--read-write-statistics", read_opts.read_write_statistics, "Perform read then write when doing statistics test");
    auto text_read = read_option->add_option("--text", read_opts.text, "Read the text api for variable")->expected(0, -1)->capture_default_str();
    read_option->add_flag("-t,--host-time-seconds",read_opts.host_time, "Print host read time");
    read_option->add_flag("--csv", read_opts.csv, "Convenience to set --no-list, --host-time-seconds, and --timestamp-in-seconds");
    read_option->add_flag("-r,--reconnect", read_opts.reconnect, "Try to reconnect by usb path");
    read_option->add_flag("-v,--compute-velocity", read_opts.compute_velocity, "Compute velocity from motor and joint position");
    read_option->add_option("-p,--precision", read_opts.precision, "floating point precision output")->expected(1);
    read_option->add_flag("-m,--short", read_opts.mini, "Shorter output");
    read_option->add_flag("--fast_log", read_opts.fastlog, "Print the fast log");
    auto timestamp_frequency_option = read_option->add_option("--timestamp-frequency", read_opts.timestamp_frequency_hz, "Override timestamp frequency in hz");
    auto bits_option = read_option->add_option("--bits", read_opts.bits, "Process noise and display bits, ±3σ window 100 [experimental]")->type_name("NUM_SAMPLES RANGE")->expected(0,2)->capture_default_str();
    app.add_flag("-l,--list", verbose_list, "Verbose list connected motors");
    app.add_flag("-c,--check-messages-version", check_messages_version, "Check motor messages version")->type_name("TYPE")->transform(CLI::CheckedTransformer(messages_check_map, CLI::ignore_case))->expected(0,1)->default_str("major");
    app.add_flag("--no-list", no_list, "Do not list connected motors");
    app.add_flag("-v,--version", version, "Print version information");
    app.add_flag("--list-names-only", list_names, "Print only connected motor names");
    app.add_flag("--list-path-only", list_path, "Print only connected motor paths");
    app.add_flag("--list-devpath-only", list_devpath, "Print only connected motor devpaths");
    app.add_flag("--list-serial-number-only", list_serial_number, "Print only connected motor serial numbers");
    app.add_flag("--list-devnum-only", list_devnum, "Print only usb device numbers");
    app.add_flag("--no-dfu-list", no_dfu_list, "Don't list stm devices in dfu mode");
    app.add_flag("-u,--user-space", user_space_driver, "Connect through user space usb");
    auto name_option = app.add_option("-n,--names", names, "Connect only to NAME(S)")->type_name("NAME")->expected(-1);
    app.add_flag("--allow-simulated", allow_simulated, "Allow simulated motors if not connected")->needs(name_option);
    app.add_option("-p,--paths", paths, "Connect only to PATHS(S)")->type_name("PATH")->expected(-1);
    app.add_option("-d,--devpaths", devpaths, "Connect only to DEVPATHS(S)")->type_name("DEVPATH")->expected(-1);
    app.add_option("-s,--serial_numbers", serial_numbers, "Connect only to SERIAL_NUMBERS(S)")->type_name("SERIAL_NUMBER")->expected(-1);
    auto ip_option = app.add_option("-i,--ips", ips, "Connect to IP(S). If left empty, connect to all ips specified in --json-ip-file")->type_name("IP")->expected(0,-1)->default_str("{}");
    app.add_option("-j,--json-ip-file", json_ip_file, "Use json file to map ip addresses")->type_name("JSON_FILE")->expected(1)->capture_default_str();
    app.add_flag("--no-print-unconnected", no_print_unconnected, "Don't print unconnected motors, currently only used with --ips");
    auto uart_paths_option = app.add_option("-a,--uart-paths", uart_paths, "Connect to UART_PATH(S) [BAUD_RATE]")->type_name("UART_PATH")->expected(-1);
    app.add_flag("--uart-raw", uart_raw, "Use raw protocol for UART")->needs(uart_paths_option);
    app.add_flag("--lock", lock_motors, "Lock write access to motors");
    auto set_api = app.add_option("--set-api", set_api_data, "Send API data (to set parameters)")->expected(1,-1);
    app.add_flag("--api", api_mode, "Enter API mode");
    app.add_flag("--api-timing", api_timing, "Print API response times");
    auto run_stats_option = app.add_option("--run-stats", run_stats, "Check firmware run timing")->type_name("NUM_SAMPLES")->expected(0,1)->capture_default_str();
    auto set_timeout_option = app.add_option("--set-timeout", timeout_ms, "Set timeout in ms")->expected(0,1)->capture_default_str();
    auto can_option = app.add_option("-f,--can", can_devs, "Connect to CAN_DEVS(S)")->type_name("CAN_DEV")->expected(0,-1)->capture_default_str();
    CLI11_PARSE(app, argc, argv);

    signal(SIGINT,[](int /* signum */){ signal_exit = true; });

    if (command_gpio) {
        command.misc.gpio = 1;
    }

    if (*read_option && read_opts.csv) {
        read_opts.timestamp_in_seconds = true;
        read_opts.host_time = true;
        no_list = true;
        no_print_unconnected = true;
    }

    MotorManager m(user_space_driver, check_messages_version);
    std::vector<std::shared_ptr<Motor>> motors;
    if (names.size()) {
        motors = m.get_motors_by_name(names, true, allow_simulated);
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
    if (*ip_option) {
        std::vector<std::string> ip_aliases;
        // translate name aliases to ips via json file
        if (access(json_ip_file.c_str(), F_OK) == 0) {
            try {
                auto motor_ips = nlohmann::ordered_json::parse(std::ifstream(json_ip_file));
                if (ips.size() == 0) {
                    // connect to all ips
                    for(auto &ip : motor_ips.items()) {
                        ips.push_back(ip.key());
                    }
                }
                for (auto &address : ips) {
                    if (motor_ips.find(address) != motor_ips.end()) {
                        ip_aliases.push_back(address);
                        address = motor_ips[address].get<std::string>();
                        
                    } else {
                        ip_aliases.push_back("");
                    }
                }
                
            } catch (nlohmann::json::parse_error &e) {
                std::cerr << "Error: json file " << json_ip_file << " could not be parsed: " << e.what() << std::endl;
            }
        } else {
            if (json_ip_file != json_ip_file_default) {
                std::cerr << "Error: json file " << json_ip_file << " not accessible" << std::endl;
            }
        }

        auto tmp_motors = m.get_motors_by_ip(ips, true, !no_print_unconnected, false, ip_aliases);
        motors.insert(motors.end(), tmp_motors.begin(), tmp_motors.end());
    }
    if (uart_paths.size()) {
        uint32_t baud_rate = 0;
        if (uart_paths.size() > 1) {
            char *p;
            long i = std::strtol(uart_paths.back().c_str(), &p, 10);
            if (*p == 0) {
                baud_rate = i;
                uart_paths.pop_back();
            }
        }
        std::vector<std::shared_ptr<Motor>> tmp_motors;
        if (baud_rate) {
            tmp_motors = m.get_motors_uart_by_devpath(uart_paths, uart_raw, baud_rate);
        } else {
            tmp_motors = m.get_motors_uart_by_devpath(uart_paths, uart_raw);
        }
        motors.insert(motors.end(), tmp_motors.begin(), tmp_motors.end());
    }

    if (*can_option) {
        std::vector<std::shared_ptr<Motor>> tmp_motors;
        tmp_motors = m.get_motors_can(can_devs);
        motors.insert(motors.end(), tmp_motors.begin(), tmp_motors.end());
    }
    
    bool messages_mismatch = false;
    std::string messages_mismatch_error;
    // remove null motors
    {
        auto i = std::begin(motors);
        while (i != std::end(motors)) {
            if (!*i) {
                i = motors.erase(i);
            } else {
                ++i;
            }
        }
        if (motors.size() > 0) {
            try {
                m.set_motors(motors);
            } catch (std::runtime_error &e) {
                messages_mismatch = true;
                messages_mismatch_error = e.what();
                m.check_messages_version(Motor::MessagesCheck::NONE);
                m.set_motors(motors);
            }
        }
    }
    
    if (!names.size() && !paths.size() && !devpaths.size() && !serial_numbers.size() && !uart_paths.size() && !*ip_option && !*can_option) {
        try {
            motors = m.get_connected_motors();
        } catch (std::runtime_error &e) {
            messages_mismatch = true;
            messages_mismatch_error = e.what();
            m.check_messages_version(Motor::MessagesCheck::NONE);
            motors = m.get_connected_motors();
        }
    } else {
        no_dfu_list = true;
    }

    if (lock_motors) {
        m.lock();
    }

    if (*set_timeout_option) {
        for (auto m : motors) {
            m->set_timeout_ms(timeout_ms);
        }
    }

    if (version) {
        std::cout << "motor_util version: " << RT_VERSION_STRING << std::endl;
    }

    if (!no_list) {
        std::vector<std::shared_ptr<MotorDescription>> motor_list;
        for (auto m : motors) {
            motor_list.emplace_back(m);
        }
        std::vector<std::string> dfu_devices;
        if (!no_dfu_list) {
            dfu_devices = udev_list_dfu();
            if (dfu_devices.size() > 0) {
                for (int i=0; i<dfu_devices.size(); i++) {
                    motor_list.emplace_back(std::make_unique<DFUDevice>(dfu_devices[i]));
                }
            }
        }

        int name_width = 6;
        int serial_number_width = 15;
        int version_width = 9;
        int path_width = 6;
        int dev_path_width = 5;
        int device_num_width = 8;
        int board_name_width = verbose_list ? 12 : 0;
        int board_rev_width = verbose_list ? 11 : 0;
        int board_num_width = verbose_list ? 11 : 0;
        int config_width = verbose_list ? 7 : 0;
        if (motor_list.size() > 0) {
            for (auto m : motor_list) {
                name_width = std::max(name_width, (int) m->name().size()+3);
                path_width = std::max(path_width, (int) m->base_path().size()+3);
                dev_path_width = std::max(dev_path_width, (int) m->dev_path().size());
                version_width = std::max(version_width, (int) (verbose_list ? m->version() : m->short_version()).size()+3);
                if (verbose_list) {
                    board_name_width = std::max(board_name_width, (int) m->board_name().size()+3);
                    board_rev_width = std::max(board_rev_width, (int) m->board_rev().size()+3);
                    board_num_width = std::max(board_num_width, (int) m->board_num().size()+3);
                    config_width = std::max(config_width, (int) m->config().size()+3);
                }
            }
        }
        if (list_names || list_path || list_devpath || list_serial_number || list_devnum) {
              if (motor_list.size() > 0) {
                    for (auto m : motors) {
                        if (list_names) {
                            std::cout << m->name();
                        } else if (list_path) {
                            std::cout << m->base_path();
                        } else if (list_devpath) {
                            std::cout << m->dev_path();
                        } else if (list_serial_number)  {
                            std::cout << m->serial_number();
                        } else if (list_devnum)  {
                            std::cout << +m->devnum();
                        }
                        std::cout << std::endl;
                    }
              }
        } else {
            std::cout << motors.size() << " connected motor" << (motors.size() == 1 ? "" : "s");
            if (dfu_devices.size() > 0) {
                std::cout << ", " << dfu_devices.size() << " connected dfu device" << (dfu_devices.size() == 1 ? "" : "s");
            }
            std::cout << std::endl;
            if (motor_list.size() > 0) {
                std::cout << std::setw(dev_path_width) << "Dev" << std::setw(name_width) << "Name"
                            << std::setw(serial_number_width) << " Serial number"
                            << std::setw(version_width) << "Version" << std::setw(path_width) << std::left << "  Path" << std::right << std::setw(device_num_width) << "Devnum";
                if (verbose_list) {
                    std::cout << std::setw(board_name_width) << "Board name"
                        << std::setw(board_rev_width) << "Board rev"
                        << std::setw(board_num_width) << "Board num"
                        << std::setw(config_width) << "Config";
                }             
                std::cout << std::endl;
                std::cout << std::setw(dev_path_width + name_width + serial_number_width + version_width + path_width + device_num_width + board_name_width + board_rev_width + board_num_width + config_width) << std::setfill('-') << "" << std::setfill(' ') << std::endl;
                for (auto m : motor_list) {
                    std::cout << std::setw(dev_path_width) << m->dev_path()
                            << std::setw(name_width) << m->name()
                            << std::setw(serial_number_width) << m->serial_number()
                            << std::setw(version_width) << (verbose_list ? m->version() : m->short_version())
                            << std::setw(path_width) << std::left << "  " + m->base_path() << std::right
                            << std::setw(device_num_width) << (int) m->devnum();
                    if (verbose_list) {
                        std::cout << std::setw(board_name_width) << m->board_name()
                            << std::setw(board_rev_width) << m->board_rev()
                            << std::setw(board_num_width) << m->board_num()
                            << std::setw(config_width) << m->config();
                    }        
                    std::cout << std::endl;
                }
            }
        }
    }



    if (messages_mismatch) {
        std::cerr << messages_mismatch_error << std::endl;
        return 1;
    }

    if (*run_stats_option && motors.size()) {
        std::cout << "name, max_fast_loop_cycles, max_fast_loop_period, max_main_loop_cycles, max_main_loop_period, " << 
                "mean_fast_loop_cycles, mean_fast_loop_period, mean_main_loop_cycles, mean_main_loop_period" << std::endl;
        for (auto m : motors) {
            auto max_api_val = [run_stats](TextAPIItem a) {
                int out = 0;
                for (int i=0;i<run_stats;i++) {
                    out = std::max(std::atoi(a.get().c_str()), out);
                } 
                return out;
            };
            auto mean_api_val = [run_stats](TextAPIItem a) {
                double out = 0;
                for (int i=0;i<run_stats;i++) {
                    out += static_cast<double>(std::atoi(a.get().c_str()))/run_stats;
                } 
                return out;
            };
            std::cout << m->name() << ", ";
            std::cout << max_api_val((*m)["t_exec_fastloop"]) << ", ";
            std::cout << max_api_val((*m)["t_period_fastloop"]) << ", ";
            std::cout << max_api_val((*m)["t_exec_mainloop"]) << ", ";
            std::cout << max_api_val((*m)["t_period_mainloop"]) << ", ";
            std::cout << mean_api_val((*m)["t_exec_fastloop"]) << ", ";
            std::cout << mean_api_val((*m)["t_period_fastloop"]) << ", ";
            std::cout << mean_api_val((*m)["t_exec_mainloop"]) << ", ";
            std::cout << mean_api_val((*m)["t_period_mainloop"]) << std::endl;
        }
    }

    if (*set && motors.size()) {
        m.set_commands(std::vector<Command>(motors.size(), command));
        std::cout << "Writing commands: \n" << m.command_headers() << std::endl << m.commands() << std::endl;
        m.write_saved_commands();
    }

    if (api_mode || (*read_option && *text_read || (*read_option && read_opts.fastlog))) {
        if (motors.size() != 1) {
            std::cout << "Select one motor to use api mode" << std::endl;
            return 1;
        }
    }

    if (*set_api && motors.size()) {
        char c[MAX_API_DATA_SIZE+1];
        for (auto &api_str : set_api_data) {
            std::cout << api_str << std::endl;
            for (auto motor : m.motors()) {
                auto tstart = std::chrono::steady_clock::now();
                auto nbytes = motor->motor_text()->writeread(api_str.c_str(), api_str.size(), c, MAX_API_DATA_SIZE);
                auto tend = std::chrono::steady_clock::now();
                if (api_timing) {
                    std::cout << "(" << std::chrono::duration_cast<std::chrono::microseconds>(tend - tstart).count() << " us) ";
                }
                if (nbytes < 0) {
                    std::cout << motor->name() << ": api_error" << std::endl;
                } else {
                    c[nbytes] = 0;
                    std::cout << motor->name() << ": " << c << std::endl;
                }
            }
        }
    }

    if (api_mode) {
        Keyboard k;
        char data[MAX_API_DATA_SIZE+1];
        while(!signal_exit) {
            if (k.new_key()) {
                char c = k.get_char();
                auto tstart = std::chrono::steady_clock::now();
                auto nbytes = m.motors()[0]->motor_text()->writeread(&c, 1, data, MAX_API_DATA_SIZE);
                auto tend = std::chrono::steady_clock::now();
                if (api_timing && c == '\n') {
                    std::cout << "(" << std::chrono::duration_cast<std::chrono::microseconds>(tend - tstart).count() << " us) ";
                }
                if (nbytes < 0) {
                    std::cout << "api_error" << std::endl;
                } else {
                    data[nbytes] = 0;
                    std::cout << data << std::flush;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    try {

    if (*read_option) {
        if (m.motors().size() == 0) {
            throw std::runtime_error("No motors connected");
        }
        
        m.set_reconnect(read_opts.reconnect);

        if (read_opts.fastlog) {
            std::cout << m.motors()[0]->get_fast_log();
            return 0;
        }
        
        if (*text_read) {
            std::vector<TextAPIItem> log;
            for (auto &s : read_opts.text) {
                if (s == "log") {
                    // only log
                    log = {(*m.motors()[0])[s]};
                    break;
                }
                log.push_back((*m.motors()[0])[s]);
                std::cout << s;
                if (&s == &read_opts.text.back()) {
                    std::cout << std::endl;
                } else {
                    std::cout << ", ";
                }
            }
            RealtimeThread text_thread(static_cast<uint32_t>(read_opts.frequency_hz), [&](){
                for (auto &l : log) {
                    auto tstart = std::chrono::steady_clock::now();
                    auto str = l.get();
                    auto tend = std::chrono::steady_clock::now();
                    if (str != "log end") {
                        if (api_timing) {
                            std::cout << "(" << std::chrono::duration_cast<std::chrono::microseconds>(tend - tstart).count() << " us) ";
                        }
                        std::cout << str;
                        if (*bits_option) {
                            static std::map<TextAPIItem*, Statistics> s;
                            s.insert({&l, read_opts.bits[0]});
                            double range = read_opts.bits[1];
                            s[&l].push(fabs(std::stod(str)));
                            std::cout << ", " << log2(range/6/s[&l].get_stddev());
                        }
                        if (&l == &log.back()) {
                            std::cout << std::endl;
                        } else {
                            std::cout << ", ";
                        }
                    }
                }
            });
            text_thread.run();
            while(!signal_exit) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            text_thread.done();
        } else {
            std::vector<double> cpu_frequency_hz(motors.size());
            if (read_opts.statistics || read_opts.read_write_statistics) {
                std::cout << "host_time_ns period_avg_ns period_std_dev_ns period_min_ns period_max_ns read_time_avg_ns read_time_std_dev_ns read_time_min_ns read_time_max_ns";
                if (read_opts.read_write_statistics) {
                   std::cout << " avg_hops";
                }
                std::cout << std::endl;
            } else if (*bits_option) {
                std::cout << "motor_encoder, output_encoder, iq" << std::endl;
            } else {
                if (read_opts.host_time) {
                    std::cout << "t_host,";
                }
                if (read_opts.timestamp_in_seconds || read_opts.compute_velocity) {
                    for (int i=0;i<motors.size();i++) {
                        if (*timestamp_frequency_option) {
                            cpu_frequency_hz[i] = read_opts.timestamp_frequency_hz;
                        } else {
                            cpu_frequency_hz[i] = motors[i]->get_cpu_frequency();
                        }
                    }
                }
                if (read_opts.timestamp_in_seconds) {
                    int length = motors.size();
                    for (int i=0;i<length;i++) {
                        std::cout << "t_seconds" << i << ", ";
                    }
                }
                std::cout << m.status_headers(read_opts.mini);
                if (read_opts.compute_velocity) {
                    for (int i=0;i<motors.size();i++) {
                        std::cout << "motor_velocity_computed" << i << ", ";
                    }
                    for (int i=0;i<motors.size();i++) {
                        std::cout << "joint_velocity_computed" << i << ", ";
                    }
                }
                std::cout << std::endl;
            }
            auto start_time = std::chrono::steady_clock::now();
            auto next_time = start_time;
            auto loop_start_time = start_time;
            int64_t period_ns = static_cast<int64_t>(1e9/read_opts.frequency_hz);
            Statistics exec(100), period(100), hops(100*m.motors().size());
            if (read_opts.nonblock) {
                for (auto &m : m.motors()) {
                    m->set_nonblock();
                }
            }
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
                if (read_opts.ppoll) {
                    int retval = m.multipoll(1000*1000);
                    if (retval < 0) {
                        std::cerr << "multipoll time out " << retval << std::endl;
                    }
                }
                
                auto status = m.read();
                auto exec_time = std::chrono::steady_clock::now();

                if (*bits_option) {
                    static Statistics motor_encoder(read_opts.bits[0]), output_encoder(read_opts.bits[0]), iq(read_opts.bits[0]);
                    static double mcpr = fabs(std::stod((*m.motors()[0])["mcpr"].get()));
                    static double irange = fabs(std::stod((*m.motors()[0])["irange"].get()));
                    motor_encoder.push(status[0].motor_encoder);
                    output_encoder.push(status[0].joint_position);
                    iq.push(status[0].iq);
                    std::cout << log2(mcpr/6/motor_encoder.get_stddev()) << ", "
                              << log2(2*M_PI/6/output_encoder.get_stddev()) << ", "
                              << log2(irange/6/iq.get_stddev()) << std::endl;
                } else if (read_opts.statistics || read_opts.read_write_statistics) {
                    static int i = 0;
                    i++;
                    auto last_exec = std::chrono::duration_cast<std::chrono::nanoseconds>(exec_time - loop_start_time).count();
                    auto last_start = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - start_time).count();
                    auto last_period = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - last_loop_start_time).count();
                    exec.push(last_exec);
                    period.push(last_period);
                    if (i > 100) {
                        i = 0;
                        auto width = 12;
                        std::cout << std::fixed << std::setprecision(0) << std::setw(width) << last_start << std::setw(width) << floor(period.get_mean()) << std::setw(width) << 
                        period.get_stddev() << std::setw(width) << period.get_min()  << std::setw(width) << period.get_max() << std::setw(width) <<
                        floor(exec.get_mean()) << std::setw(width) <<  exec.get_stddev() << std::setw(width) << exec.get_min() << std::setw(width) << exec.get_max();
                        if (read_opts.read_write_statistics) {
                            std::cout << std::setprecision(3) << std::setw(width) << hops.get_mean();
                        }
                        std::cout << std::endl;
                    }
                    if (read_opts.read_write_statistics) { 
                        for (auto s : status) {
                            hops.push(m.get_auto_count() - s.host_timestamp_received);
                        }
                        m.set_auto_count();
                        m.write_saved_commands();
                    }
                } else {
                    std::cout << std::fixed;
                    std::cout << std::setprecision(9);
                    if (read_opts.host_time) {
                        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start_time - start_time).count()/1e9 << ", ";
                    }
                    if (read_opts.timestamp_in_seconds) {
                        static auto last_status = status;
                        static double *t_seconds = new double[status.size()]();
                        for (int i = 0; i < status.size(); i++) {
                            uint32_t dt = status[i].mcu_timestamp - last_status[i].mcu_timestamp;
                            t_seconds[i] += dt/cpu_frequency_hz[i];
                            std::cout << t_seconds[i] << ", ";
                        }
                        last_status = status;
                    }
                    std::cout << std::setprecision(read_opts.precision);
                    if (read_opts.mini) {
                        std::cout << short_status(status);
                    } else {
                        std::cout << status;
                    }
                    if (read_opts.compute_velocity) {
                        static auto last_status = status;
                        for (int i = 0; i < status.size(); i++) {
                            double dt = (status[i].mcu_timestamp - last_status[i].mcu_timestamp)/cpu_frequency_hz[i];
                            double velocity = (status[i].motor_position - last_status[i].motor_position)/dt;
                            std::cout << std::setw(8) << velocity << ", ";
                        }
                        for (int i = 0; i < status.size(); i++) {
                            double dt = (status[i].mcu_timestamp - last_status[i].mcu_timestamp)/cpu_frequency_hz[i];
                            double velocity = (status[i].joint_position - last_status[i].joint_position)/dt;
                            std::cout << std::setw(8) << velocity << ", ";
                        }
                        last_status = status;
                    }
                    std::cout << std::endl;
                }
                if (read_opts.nonblock) {
                    m.start_nonblocking_read();
                }

                // option to not sleep
                // while (std::chrono::steady_clock::now() < next_time);
                std::this_thread::sleep_until(next_time);
            }
        }
    }

    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
