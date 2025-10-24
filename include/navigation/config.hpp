/**
 * Configuration management for navigation system.
 */
#ifndef NAVIGATION_CONFIG_HPP
#define NAVIGATION_CONFIG_HPP

#include <string>
#include <map>
#include <memory>
#include <fstream>

namespace navigation {

/**
 * Configuration parameters for navigation system.
 */
struct NavigationConfig {
    // Grid parameters
    int grid_size = 400;
    double grid_resolution = 0.25;

    // Navigation parameters
    double slalom_distance = 4.0;
    double clear_distance = 4.0;
    double loiter_distance = 10.0;

    // Start position
    double start_x = 50.0;
    double start_y = -100.0;
    double start_psi = 0.0;

    // Color mappings
    std::map<int, std::string> buoy_colors;

    NavigationConfig() {
        initializeDefaults();
    }

    void initializeDefaults() {
        buoy_colors = {
            {0, "free"}, {10, "l"}, {20, "r"}, {30, "g"}, {40, "y"},
            {50, "b"}, {60, "m"}, {70, "c"}, {80, "k"}, {90, "w"}, {100, "u"}
        };
    }
};

/**
 * Manages configuration loading and saving.
 */
class ConfigManager {
private:
    std::string config_file_;
    NavigationConfig config_;

public:
    explicit ConfigManager(const std::string& config_file = "navigation_config.json");

    /**
     * Load configuration from file.
     */
    NavigationConfig loadConfig();

    /**
     * Save current configuration to file.
     */
    void saveConfig();

    /**
     * Get current configuration.
     */
    const NavigationConfig& getConfig() const { return config_; }
    NavigationConfig& getConfig() { return config_; }

    /**
     * Update configuration parameter.
     */
    template<typename T>
    void updateConfig(const std::string& key, const T& value);
};

} // namespace navigation

#endif // NAVIGATION_CONFIG_HPP
