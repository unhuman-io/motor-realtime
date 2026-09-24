#include "CLI11.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <json.hpp>
#include "motor_eth_l2.h"

namespace obot {

// Helper to print all flags and subcommands for a given CLI::App level
void print_options(const CLI::App* app) {
    // Print individual short and long flags
    for (const auto* opt : app->get_options()) {
        for (const auto& sname : opt->get_snames()) {
            std::cout << "-" << sname << " ";
        }
        for (const auto& lname : opt->get_lnames()) {
            std::cout << "--" << lname << " ";
        }
    }
    // Print subcommand names
    for (const auto* sub : app->get_subcommands()) {
        if (!sub->get_name().empty()) {
            std::cout << sub->get_name() << " ";
        }
    }
}

bool handle_autocomplete(int argc, char** argv, CLI::App& main_app) {
    // We are looking for: motor_util --autocomplete <CWORD> <COMP_WORDS...>
    if (argc >= 3 && std::string(argv[1]) == "--autocomplete") {
        int cword = std::stoi(argv[2]);
        std::vector<std::string> comp_words;
        for (int i = 3; i < argc; ++i) {
            comp_words.push_back(argv[i]);
        }

        std::string last_word = (cword > 0 && cword <= comp_words.size()) ? comp_words[cword - 1] : "";

        // 1. Check for context-specific dynamic lists (e.g., IPs from JSON)
        if (last_word == "-i" || last_word == "--ips") {
            std::string json_file = "/etc/motor_util/device_ip_map.json"; // Adjust path as needed
            std::ifstream file(json_file);
            if (file.good()) {
                try {
                    auto motor_ips = nlohmann::ordered_json::parse(file);
                    for(auto &ip : motor_ips.items()) {
                        std::cout << ip.key() << " ";
                    }
                } catch (...) {}
            }
            std::cout << "\n";
            return true;
        }

        if (last_word == "-e" || last_word == "--eth-l2") {
            // Call your native C++ get_eth_interfaces() here if needed
            auto e = get_eth_interfaces();
            for (auto &i : e) std::cout << i << " ";
            std::cout << "\n";
            return true;
        }

        // 2. Drill down into nested subcommands based on fully typed words
        CLI::App* active_app = &main_app;
        
        // Only evaluate up to cword (exclusive). 
        // The word at `cword` is currently being typed and should not trigger a context switch!
        for (int i = 0; i < cword; ++i) {
            if (i >= comp_words.size()) break;
            
            try {
                CLI::App* sub = active_app->get_subcommand(comp_words[i]);
                if (sub != nullptr) {
                    active_app = sub; // Successfully moved one level deeper (e.g., main -> set -> current)
                }
            } catch (const CLI::OptionNotFound&) {
                // Not a subcommand (could be a flag or value), continue checking
                continue;
            }
        }

        // 3. Output options and subcommands based on the deepest resolved context
        print_options(active_app);
        std::cout << "\n";
        
        return true; // Autocomplete handled, exit cleanly
    }
    return false;
}

} // namespace obot
