#include "navigation/config.hpp"
#include <iostream>
#include <sstream>

namespace navigation {

ConfigManager::ConfigManager(const std::string& config_file)
    : config_file_(config_file) {
    config_.initializeDefaults();
}

NavigationConfig ConfigManager::loadConfig() {
    std::ifstream file(config_file_);
    if (file.is_open()) {
        // Simple config loading - in production, use nlohmann/json or similar
        std::cout << "Note: JSON parsing not implemented in basic version.\n";
        std::cout << "Using default configuration.\n";
        file.close();
    }
    return config_;
}

void ConfigManager::saveConfig() {
    std::ofstream file(config_file_);
    if (file.is_open()) {
        // Simple config saving - in production, use nlohmann/json or similar
        file << "{\n";
        file << "  \"grid_size\": " << config_.grid_size << ",\n";
        file << "  \"grid_resolution\": " << config_.grid_resolution << ",\n";
        file << "  \"slalom_distance\": " << config_.slalom_distance << ",\n";
        file << "  \"clear_distance\": " << config_.clear_distance << ",\n";
        file << "  \"loiter_distance\": " << config_.loiter_distance << ",\n";
        file << "  \"start_x\": " << config_.start_x << ",\n";
        file << "  \"start_y\": " << config_.start_y << ",\n";
        file << "  \"start_psi\": " << config_.start_psi << "\n";
        file << "}\n";
        file.close();
    } else {
        std::cerr << "Error: Could not save config file\n";
    }
}

template<typename T>
void ConfigManager::updateConfig(const std::string& key, const T& value) {
    std::cerr << "Warning: Dynamic config updates not fully implemented\n";
}

} // namespace navigation
