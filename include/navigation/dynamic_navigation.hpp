#pragma once

#include "navigation/dynamic_kdtree.hpp"
#include "navigation/buoy_detection.hpp"
#include "navigation/config.hpp"
#include <vector>
#include <memory>
#include <chrono>
#include <random>

namespace navigation {

using Point = std::pair<double, double>;

/**
 * Simulates real-time sensor data for dynamic navigation testing.
 */
class SensorSimulator {
private:
    std::mt19937 rng_;
    std::vector<Buoy> static_buoys_;
    std::vector<std::tuple<Point, double, double>> moving_obstacles_;  // position, velocity_x, velocity_y
    double simulation_time_;
    
public:
    SensorSimulator();
    
    // Initialize environment
    void addStaticBuoy(double x, double y, int color);
    void addMovingObstacle(double x, double y, double velocity_x, double velocity_y);
    
    // Simulate sensor readings
    std::vector<Buoy> getCurrentBuoys() const;
    std::vector<Point> getCurrentObstacles() const;
    
    // Update simulation
    void updateSimulation(double dt);
    void setSimulationTime(double time) { simulation_time_ = time; }
    double getSimulationTime() const { return simulation_time_; }
};

/**
 * Dynamic navigation system that handles moving obstacles and continuous updates.
 */
class DynamicNavigationSystem {
private:
    std::unique_ptr<DynamicKDTree> kdtree_;
    std::unique_ptr<SensorSimulator> sensor_sim_;
    std::unique_ptr<ConfigManager> config_manager_;
    NavigationConfig config_;
    
    Point current_position_;
    Point target_position_;
    std::vector<Point> current_path_;
    double path_update_interval_;
    double last_path_update_;
    
    // Navigation state
    bool is_navigating_;
    double navigation_speed_;
    double avoidance_radius_;
    
public:
    DynamicNavigationSystem();
    
    // Environment setup
    void initializeEnvironment(const std::string& difficulty);
    void addStaticObstacle(double x, double y);
    void addMovingObstacle(double x, double y, double velocity_x, double velocity_y);
    
    // Navigation control
    void startNavigation(const Point& start, const Point& goal);
    void stopNavigation();
    bool isNavigating() const { return is_navigating_; }
    
    // Real-time updates
    void updateNavigation(double dt);
    void updateEnvironment();
    void replanPath();
    
    // Pathfinding algorithms using dynamic KD-Tree
    std::vector<Point> findDynamicPath(const Point& start, const Point& goal);
    std::vector<Point> computeDynamicConvexHull();
    std::tuple<double, std::vector<int>> solveDynamicTSP(const Point& start);
    
    // Status and information
    Point getCurrentPosition() const { return current_position_; }
    Point getTargetPosition() const { return target_position_; }
    std::vector<Point> getCurrentPath() const { return current_path_; }
    size_t getObstacleCount() const;
    double getSimulationTime() const { return sensor_sim_->getSimulationTime(); }
    
    // Simulation control
    void runSimulation(double duration, double dt = 0.1);
    void printStatus() const;
    
    // Debugging
    void printEnvironment() const;
    void printKDTreeStats() const;
};

} // namespace navigation
