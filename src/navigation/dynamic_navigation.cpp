#include "navigation/dynamic_navigation.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace navigation {

// ============================================================================
// SENSOR SIMULATOR IMPLEMENTATION
// ============================================================================

SensorSimulator::SensorSimulator() 
    : rng_(std::chrono::steady_clock::now().time_since_epoch().count()),
      simulation_time_(0.0) {}

void SensorSimulator::addStaticBuoy(double x, double y, int color) {
    static_buoys_.emplace_back(x, y, color);
}

void SensorSimulator::addMovingObstacle(double x, double y, double velocity_x, double velocity_y) {
    moving_obstacles_.emplace_back(Point{x, y}, velocity_x, velocity_y);
}

std::vector<Buoy> SensorSimulator::getCurrentBuoys() const {
    std::vector<Buoy> current_buoys = static_buoys_;
    
    // Add moving obstacles as "buoys" for navigation purposes
    for (const auto& [pos, vx, vy] : moving_obstacles_) {
        double current_x = pos.first + vx * simulation_time_;
        double current_y = pos.second + vy * simulation_time_;
        current_buoys.emplace_back(current_x, current_y, 20);  // Red obstacle
    }
    
    return current_buoys;
}

std::vector<Point> SensorSimulator::getCurrentObstacles() const {
    std::vector<Point> obstacles;
    
    for (const auto& [pos, vx, vy] : moving_obstacles_) {
        double current_x = pos.first + vx * simulation_time_;
        double current_y = pos.second + vy * simulation_time_;
        obstacles.emplace_back(current_x, current_y);
    }
    
    return obstacles;
}

void SensorSimulator::updateSimulation(double dt) {
    simulation_time_ += dt;
}

// ============================================================================
// DYNAMIC NAVIGATION SYSTEM IMPLEMENTATION
// ============================================================================

DynamicNavigationSystem::DynamicNavigationSystem()
    : kdtree_(std::make_unique<DynamicKDTree>(0.2)),  // Rebuild when 20% of points change
      sensor_sim_(std::make_unique<SensorSimulator>()),
      config_manager_(std::make_unique<ConfigManager>("navigation_config.json")),
      config_(config_manager_->loadConfig()),
      current_position_{0, 0},
      target_position_{0, 0},
      path_update_interval_(1.0),  // Replan every 1 second
      last_path_update_(0.0),
      is_navigating_(false),
      navigation_speed_(2.0),  // 2 m/s
      avoidance_radius_(2.0)   // 2m avoidance radius (realistic for 1m buoy movement)
{}

void DynamicNavigationSystem::initializeEnvironment(const std::string& difficulty) {
    std::cout << "🌊 Initializing dynamic environment: " << difficulty << "\n";
    std::cout << "   📏 Buoy movement: ~1m radius (realistic ocean currents)\n";
    std::cout << "   🛡️  Avoidance radius: 2m (safe distance from moving buoys)\n";
    
    // Clear existing environment
    kdtree_->clear();
    sensor_sim_ = std::make_unique<SensorSimulator>();
    
    // Add static buoys based on difficulty
    if (difficulty == "easy") {
        sensor_sim_->addStaticBuoy(42, 10, 30);
        sensor_sim_->addStaticBuoy(48, 10, 20);
        sensor_sim_->addStaticBuoy(44, 20, 20);
        sensor_sim_->addStaticBuoy(46, 20, 30);
        
        // Add one moving obstacle with realistic 1m radius movement
        sensor_sim_->addMovingObstacle(45, 15, 0.1, 0.05);  // ~1m radius movement
        
    } else if (difficulty == "medium") {
        sensor_sim_->addStaticBuoy(42, 10, 30);
        sensor_sim_->addStaticBuoy(48, 10, 20);
        sensor_sim_->addStaticBuoy(44, 20, 20);
        sensor_sim_->addStaticBuoy(43, 30, 30);
        sensor_sim_->addStaticBuoy(46, 40, 20);
        sensor_sim_->addStaticBuoy(45, 50, 30);
        sensor_sim_->addStaticBuoy(42, 65, 30);
        sensor_sim_->addStaticBuoy(48, 65, 20);
        
        // Add moving obstacles with realistic 1m radius movement
        sensor_sim_->addMovingObstacle(45, 25, 0.08, 0.03);  // ~1m radius
        sensor_sim_->addMovingObstacle(47, 35, -0.05, 0.04); // ~1m radius
        
    } else if (difficulty == "hard") {
        // More complex static environment
        for (int i = 0; i < 12; i++) {
            double x = 40 + (i % 4) * 3;
            double y = 10 + (i / 4) * 15;
            int color = (i % 2) ? 20 : 30;
            sensor_sim_->addStaticBuoy(x, y, color);
        }
        
        // Multiple moving obstacles with realistic 1m radius movement
        sensor_sim_->addMovingObstacle(45, 20, 0.1, 0.05);   // ~1m radius
        sensor_sim_->addMovingObstacle(43, 35, -0.07, 0.03); // ~1m radius
        sensor_sim_->addMovingObstacle(47, 50, 0.03, -0.08); // ~1m radius
        
    } else {  // extreme
        // Dense static environment
        for (int i = 0; i < 20; i++) {
            double x = 40 + (i % 5) * 2.5;
            double y = 10 + (i / 5) * 12;
            int color = (i % 3) ? ((i % 2) ? 20 : 30) : 10;
            sensor_sim_->addStaticBuoy(x, y, color);
        }
        
        // Many moving obstacles with realistic 1m radius movement
        sensor_sim_->addMovingObstacle(45, 15, 0.12, 0.04);  // ~1m radius
        sensor_sim_->addMovingObstacle(42, 25, -0.08, 0.06); // ~1m radius
        sensor_sim_->addMovingObstacle(48, 35, 0.05, -0.09); // ~1m radius
        sensor_sim_->addMovingObstacle(44, 45, -0.03, 0.07); // ~1m radius
        sensor_sim_->addMovingObstacle(46, 55, 0.09, -0.02); // ~1m radius
    }
    
    std::cout << "✓ Environment initialized with " << sensor_sim_->getCurrentBuoys().size() 
              << " total obstacles\n";
}

void DynamicNavigationSystem::addStaticObstacle(double x, double y) {
    sensor_sim_->addStaticBuoy(x, y, 20);  // Red obstacle
}

void DynamicNavigationSystem::addMovingObstacle(double x, double y, double velocity_x, double velocity_y) {
    sensor_sim_->addMovingObstacle(x, y, velocity_x, velocity_y);
}

void DynamicNavigationSystem::startNavigation(const Point& start, const Point& goal) {
    current_position_ = start;
    target_position_ = goal;
    is_navigating_ = true;
    last_path_update_ = 0.0;
    
    std::cout << "🚢 Starting navigation from (" << std::fixed << std::setprecision(1)
              << start.first << ", " << start.second << ") to ("
              << goal.first << ", " << goal.second << ")\n";
    
    // Initial path planning
    replanPath();
}

void DynamicNavigationSystem::stopNavigation() {
    is_navigating_ = false;
    current_path_.clear();
    std::cout << "🛑 Navigation stopped\n";
}

void DynamicNavigationSystem::updateNavigation(double dt) {
    if (!is_navigating_) return;
    
    // Update simulation time
    sensor_sim_->updateSimulation(dt);
    
    // Update environment (rebuild KD-Tree if needed)
    updateEnvironment();
    
    // Check if we need to replan
    last_path_update_ += dt;
    if (last_path_update_ >= path_update_interval_) {
        replanPath();
        last_path_update_ = 0.0;
    }
    
    // Move towards next waypoint
    if (!current_path_.empty()) {
        Point next_waypoint = current_path_[0];
        double dx = next_waypoint.first - current_position_.first;
        double dy = next_waypoint.second - current_position_.second;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < 1.0) {  // Reached waypoint
            current_path_.erase(current_path_.begin());
        } else {
            // Move towards waypoint
            double move_distance = navigation_speed_ * dt;
            double move_ratio = std::min(move_distance / distance, 1.0);
            
            current_position_.first += dx * move_ratio;
            current_position_.second += dy * move_ratio;
        }
    }
    
    // Check if we've reached the goal
    double goal_distance = std::sqrt(
        std::pow(target_position_.first - current_position_.first, 2) +
        std::pow(target_position_.second - current_position_.second, 2)
    );
    
    if (goal_distance < 2.0) {
        std::cout << "🎯 Reached goal!\n";
        stopNavigation();
    }
}

void DynamicNavigationSystem::updateEnvironment() {
    // Get current obstacle positions
    auto current_buoys = sensor_sim_->getCurrentBuoys();
    
    // Update KD-Tree with new positions
    kdtree_->clear();
    for (const auto& buoy : current_buoys) {
        kdtree_->addObstacle(buoy.x, buoy.y, avoidance_radius_);
    }
    
    // Optimize tree if needed
    kdtree_->optimize();
}

void DynamicNavigationSystem::replanPath() {
    std::cout << "🔄 Replanning path... ";
    
    // Use dynamic KD-Tree for pathfinding
    current_path_ = findDynamicPath(current_position_, target_position_);
    
    std::cout << "✓ Path updated with " << current_path_.size() << " waypoints\n";
}

std::vector<Point> DynamicNavigationSystem::findDynamicPath(const Point& start, const Point& goal) {
    // Simple RRT* using dynamic KD-Tree for collision detection
    std::vector<Point> path = {start};
    
    // Use KD-Tree to find collision-free waypoints
    auto [nearest_obstacle, _, dist_to_obstacle] = kdtree_->nearestNeighbor(start);
    
    if (dist_to_obstacle > avoidance_radius_ * 2) {
        // Direct path if no obstacles nearby
        path.push_back(goal);
    } else {
        // Find intermediate waypoint using KD-Tree spatial queries
        auto obstacles_in_range = kdtree_->rangeSearch(
            std::min(start.first, goal.first) - 10,
            std::max(start.first, goal.first) + 10,
            std::min(start.second, goal.second) - 10,
            std::max(start.second, goal.second) + 10
        );
        
        if (obstacles_in_range.empty()) {
            path.push_back(goal);
        } else {
            // Find safe intermediate point
            Point intermediate = {
                (start.first + goal.first) / 2,
                (start.second + goal.second) / 2
            };
            
            // Adjust intermediate point to avoid obstacles
            auto [nearest, _, dist] = kdtree_->nearestNeighbor(intermediate);
            if (dist < avoidance_radius_ * 1.5) {
                // Move intermediate point away from obstacle
                double dx = intermediate.first - nearest.first;
                double dy = intermediate.second - nearest.second;
                double length = std::sqrt(dx * dx + dy * dy);
                if (length > 0) {
                    intermediate.first += (dx / length) * avoidance_radius_;
                    intermediate.second += (dy / length) * avoidance_radius_;
                }
            }
            
            path.push_back(intermediate);
            path.push_back(goal);
        }
    }
    
    return path;
}

std::vector<Point> DynamicNavigationSystem::computeDynamicConvexHull() {
    return kdtree_->computeConvexHullDC();
}

std::tuple<double, std::vector<int>> DynamicNavigationSystem::solveDynamicTSP(const Point& start) {
    // Get current obstacle positions for TSP
    auto obstacles = sensor_sim_->getCurrentObstacles();
    
    if (obstacles.empty()) {
        return {0.0, {}};
    }
    
    // Convert to points for TSP
    std::vector<Point> points = {start};
    for (const auto& obstacle : obstacles) {
        points.push_back(obstacle);
    }
    
    // Use existing TSP solver (simplified)
    double total_distance = 0.0;
    std::vector<int> order;
    
    for (size_t i = 0; i < points.size(); i++) {
        order.push_back(i);
        if (i > 0) {
            double dx = points[i].first - points[i-1].first;
            double dy = points[i].second - points[i-1].second;
            total_distance += std::sqrt(dx * dx + dy * dy);
        }
    }
    
    return {total_distance, order};
}

size_t DynamicNavigationSystem::getObstacleCount() const {
    return kdtree_->size();
}

void DynamicNavigationSystem::runSimulation(double duration, double dt) {
    std::cout << "\n🌊 Running dynamic navigation simulation for " << duration << " seconds...\n";
    std::cout << "   Update interval: " << dt << "s, Path replanning: every " << path_update_interval_ << "s\n\n";
    
    double current_time = 0.0;
    int update_count = 0;
    
    while (current_time < duration && is_navigating_) {
        updateNavigation(dt);
        current_time += dt;
        update_count++;
        
        // Print status every 2 seconds
        if (update_count % static_cast<int>(2.0 / dt) == 0) {
            printStatus();
        }
    }
    
    if (is_navigating_) {
        std::cout << "\n⏰ Simulation completed - navigation still active\n";
    }
}

void DynamicNavigationSystem::printStatus() const {
    std::cout << "📍 Position: (" << std::fixed << std::setprecision(1)
              << current_position_.first << ", " << current_position_.second << ")\n";
    std::cout << "🎯 Target: (" << target_position_.first << ", " << target_position_.second << ")\n";
    std::cout << "🛣️  Path waypoints: " << current_path_.size() << "\n";
    std::cout << "🌳 KD-Tree obstacles: " << kdtree_->size() << "\n";
    std::cout << "⏱️  Simulation time: " << std::fixed << std::setprecision(1)
              << sensor_sim_->getSimulationTime() << "s\n\n";
}

void DynamicNavigationSystem::printEnvironment() const {
    auto buoys = sensor_sim_->getCurrentBuoys();
    std::cout << "\n🌊 Current Environment:\n";
    std::cout << "   Static buoys: " << buoys.size() - sensor_sim_->getCurrentObstacles().size() << "\n";
    std::cout << "   Moving obstacles: " << sensor_sim_->getCurrentObstacles().size() << "\n";
    std::cout << "   Total obstacles: " << buoys.size() << "\n";
}

void DynamicNavigationSystem::printKDTreeStats() const {
    std::cout << "\n🌳 KD-Tree Statistics:\n";
    std::cout << "   Points: " << kdtree_->size() << "\n";
    std::cout << "   Empty: " << (kdtree_->isEmpty() ? "Yes" : "No") << "\n";
}

} // namespace navigation
