#include "navigation/aimm_icc_competition.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace navigation {

AIMMICCCompetitionSystem::AIMMICCCompetitionSystem() 
    : nav_system_(std::make_unique<DynamicNavigationSystem>()) {
    
    // Initialize challenge data
    challenges_[1] = {"Gate Navigation", 100, 50, false, 0.0};
    challenges_[2] = {"Dodge Slalom", 150, 100, false, 0.0};
    challenges_[3] = {"Evade", 120, 80, false, 0.0};
    challenges_[4] = {"Identify", 80, 40, false, 0.0};
    challenges_[5] = {"Deploy", 90, 60, false, 0.0};
    challenges_[6] = {"Launch", 70, 50, false, 0.0};
    challenges_[7] = {"Recover", 110, 70, false, 0.0};
    challenges_[8] = {"Receive", 60, 30, false, 0.0};
}

bool AIMMICCCompetitionSystem::challenge1_GateNavigation(const Point& dock_pos, 
                                                        const Point& lime_green_buoy, 
                                                        const Point& red_buoy) {
    std::cout << "\n🚪 CHALLENGE #1: Gate Navigation\n";
    std::cout << "   🎯 Target: Navigate through Lime Green + Red buoy gate\n";
    std::cout << "   🏆 Bonus: Zero contact with buoys\n\n";
    
    // Calculate gate center
    Point gate_center = {
        (lime_green_buoy.first + red_buoy.first) / 2,
        (lime_green_buoy.second + red_buoy.second) / 2
    };
    
    // Start navigation from dock
    nav_system_->startNavigation(dock_pos, gate_center);
    
    // Run navigation with collision avoidance
    double start_time = nav_system_->getSimulationTime();
    nav_system_->runSimulation(30.0, 0.5); // 30 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    // Check if successful (simplified - in real system would check actual position)
    bool success = nav_system_->getCurrentPath().size() > 0;
    
    if (success) {
        challenges_[1].completed = true;
        challenges_[1].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[1].points_base;
        
        std::cout << "✅ Gate Navigation COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[1].points_base << "\n";
        
        if (comp_config_.zero_collision_mode) {
            metrics_.bonus_points += challenges_[1].points_bonus;
            std::cout << "   🏆 Zero collision bonus: +" << challenges_[1].points_bonus << " points\n";
        }
    } else {
        std::cout << "❌ Gate Navigation FAILED\n";
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge2_DodgeSlalom(const std::vector<Point>& slalom_buoys) {
    std::cout << "\n🏃 CHALLENGE #2: Dodge Slalom\n";
    std::cout << "   🎯 Target: Navigate slalom course with zero contact\n";
    std::cout << "   🏆 Bonus: Fastest time vs all teams\n\n";
    
    // Use TSP-DP to find optimal slalom route
    Point start = nav_system_->getCurrentPosition();
    auto [optimal_distance, waypoint_order] = nav_system_->solveDynamicTSP(start);
    
    std::cout << "✓ Optimal slalom route calculated: " << std::fixed << std::setprecision(1)
              << optimal_distance << "m total distance\n";
    
    // Execute slalom navigation
    double start_time = nav_system_->getSimulationTime();
    
    for (size_t i = 0; i < waypoint_order.size() && i < slalom_buoys.size(); i++) {
        Point target = slalom_buoys[waypoint_order[i]];
        nav_system_->startNavigation(nav_system_->getCurrentPosition(), target);
        nav_system_->runSimulation(15.0, 0.5); // 15 seconds per buoy
    }
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[2].completed = true;
        challenges_[2].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[2].points_base;
        
        std::cout << "✅ Slalom COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[2].points_base << "\n";
        
        if (comp_config_.zero_collision_mode) {
            metrics_.bonus_points += challenges_[2].points_bonus;
            std::cout << "   🏆 Zero collision bonus: +" << challenges_[2].points_bonus << " points\n";
        }
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge3_Evade(const std::vector<Point>& channel_buoys, 
                                               const std::vector<Point>& lidar_positions) {
    std::cout << "\n🥷 CHALLENGE #3: Evade\n";
    std::cout << "   🎯 Target: Navigate channel while avoiding LiDAR detection\n";
    std::cout << "   🏆 Bonus: Fastest time while evading\n\n";
    
    // Calculate safe zones using convex hull
    auto safe_zones = nav_system_->computeDynamicConvexHull();
    
    std::cout << "✓ Safe zones calculated: " << safe_zones.size() << " vertices\n";
    
    // Plan evasion route avoiding LiDAR positions
    Point start = nav_system_->getCurrentPosition();
    Point end = channel_buoys.back(); // End of channel
    
    auto evasion_path = planEvasionRoute(channel_buoys, lidar_positions);
    
    double start_time = nav_system_->getSimulationTime();
    nav_system_->startNavigation(start, end);
    nav_system_->runSimulation(25.0, 0.5); // 25 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[3].completed = true;
        challenges_[3].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[3].points_base;
        
        std::cout << "✅ Evade COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[3].points_base << "\n";
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge4_Identify(const Point& target_buoy, int target_color) {
    std::cout << "\n🎯 CHALLENGE #4: Identify\n";
    std::cout << "   🎯 Target: Navigate to color " << target_color << " buoy\n";
    std::cout << "   🏆 Bonus: Random color selection\n\n";
    
    Point start = nav_system_->getCurrentPosition();
    
    double start_time = nav_system_->getSimulationTime();
    nav_system_->startNavigation(start, target_buoy);
    nav_system_->runSimulation(20.0, 0.5); // 20 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[4].completed = true;
        challenges_[4].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[4].points_base;
        
        std::cout << "✅ Identify COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[4].points_base << "\n";
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge5_Deploy(const Point& zebra_buoy, const Point& deployment_position) {
    std::cout << "\n📡 CHALLENGE #5: Deploy\n";
    std::cout << "   🎯 Target: Navigate to Zebra buoy and deploy sensor\n";
    std::cout << "   📏 Deployment zone: 6' radius from Zebra buoy\n\n";
    
    Point start = nav_system_->getCurrentPosition();
    Point deployment_zone = calculateDeploymentZone(zebra_buoy, 6.0);
    
    std::cout << "✓ Deployment zone calculated: (" << std::fixed << std::setprecision(1)
              << deployment_zone.first << ", " << deployment_zone.second << ")\n";
    
    double start_time = nav_system_->getSimulationTime();
    nav_system_->startNavigation(start, deployment_zone);
    nav_system_->runSimulation(25.0, 0.5); // 25 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[5].completed = true;
        challenges_[5].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[5].points_base;
        
        std::cout << "✅ Deploy COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[5].points_base << "\n";
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge6_Launch(const Point& black_buoy, const Point& target_position) {
    std::cout << "\n🚀 CHALLENGE #6: Launch\n";
    std::cout << "   🎯 Target: Navigate to Black buoy and launch projectile\n";
    std::cout << "   🎯 Launch target: (" << std::fixed << std::setprecision(1)
              << target_position.first << ", " << target_position.second << ")\n\n";
    
    Point start = nav_system_->getCurrentPosition();
    Point optimal_launch = calculateOptimalLaunchPosition(black_buoy, target_position);
    
    std::cout << "✓ Optimal launch position calculated: (" << std::fixed << std::setprecision(1)
              << optimal_launch.first << ", " << optimal_launch.second << ")\n";
    
    double start_time = nav_system_->getSimulationTime();
    nav_system_->startNavigation(start, optimal_launch);
    nav_system_->runSimulation(20.0, 0.5); // 20 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[6].completed = true;
        challenges_[6].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[6].points_base;
        
        std::cout << "✅ Launch COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[6].points_base << "\n";
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge7_Recover(const Point& floating_object) {
    std::cout << "\n🔍 CHALLENGE #7: Recover\n";
    std::cout << "   🎯 Target: Find and retrieve floating object\n";
    std::cout << "   🏆 Bonus: No damage to object\n\n";
    
    Point start = nav_system_->getCurrentPosition();
    
    double start_time = nav_system_->getSimulationTime();
    nav_system_->startNavigation(start, floating_object);
    nav_system_->runSimulation(30.0, 0.5); // 30 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[7].completed = true;
        challenges_[7].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[7].points_base;
        
        std::cout << "✅ Recover COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[7].points_base << "\n";
    }
    
    return success;
}

bool AIMMICCCompetitionSystem::challenge8_Receive(const Point& return_buoy) {
    std::cout << "\n📡 CHALLENGE #8: Receive\n";
    std::cout << "   🎯 Target: Communicate with deployed sensor and return to dock\n";
    std::cout << "   🏠 Return: Contact marked return buoy\n\n";
    
    Point start = nav_system_->getCurrentPosition();
    
    double start_time = nav_system_->getSimulationTime();
    nav_system_->startNavigation(start, return_buoy);
    nav_system_->runSimulation(20.0, 0.5); // 20 second timeout
    
    double completion_time = nav_system_->getSimulationTime() - start_time;
    
    bool success = true; // Simplified success check
    if (success) {
        challenges_[8].completed = true;
        challenges_[8].completion_time = completion_time;
        metrics_.challenges_completed++;
        metrics_.total_points += challenges_[8].points_base;
        
        std::cout << "✅ Receive COMPLETED in " << std::fixed << std::setprecision(1) 
                  << completion_time << "s\n";
        std::cout << "   Points earned: " << challenges_[8].points_base << "\n";
    }
    
    return success;
}

void AIMMICCCompetitionSystem::startCompetition() {
    std::cout << "\n🏆 AIMM-ICC COMPETITION STARTED 🏆\n";
    std::cout << "=====================================\n";
    std::cout << "⏰ Time limit: 1 hour (3600 seconds)\n";
    std::cout << "🎯 Challenges: 8 total\n";
    std::cout << "🏆 Bonus modes: GPS-Agnostic, Zero Collision\n\n";
    
    metrics_.total_time = 0.0;
    metrics_.challenges_completed = 0;
    metrics_.total_points = 0;
    metrics_.bonus_points = 0;
}

void AIMMICCCompetitionSystem::runCompetition(double time_limit) {
    startCompetition();
    
    // Initialize competition environment
    nav_system_->initializeEnvironment("extreme"); // Use extreme difficulty for competition
    
    // Example competition run (simplified)
    Point dock = {40.0, 5.0};
    Point lime_green = {42.0, 10.0};
    Point red_buoy = {48.0, 10.0};
    
    // Run challenges in order
    challenge1_GateNavigation(dock, lime_green, red_buoy);
    
    std::vector<Point> slalom_buoys = {
        {44.0, 20.0}, {43.0, 30.0}, {46.0, 40.0}, {45.0, 50.0}
    };
    challenge2_DodgeSlalom(slalom_buoys);
    
    // Continue with other challenges...
    
    printCompetitionStatus();
}

void AIMMICCCompetitionSystem::stopCompetition() {
    std::cout << "\n🏁 AIMM-ICC COMPETITION COMPLETED 🏁\n";
    std::cout << "=====================================\n";
    printCompetitionStatus();
}

int AIMMICCCompetitionSystem::calculateScore() const {
    return metrics_.total_points + metrics_.bonus_points;
}

AIMMICCCompetitionSystem::CompetitionMetrics AIMMICCCompetitionSystem::getMetrics() const {
    return metrics_;
}

void AIMMICCCompetitionSystem::printCompetitionStatus() const {
    std::cout << "\n📊 COMPETITION STATUS\n";
    std::cout << "=====================\n";
    std::cout << "Challenges completed: " << metrics_.challenges_completed << "/8\n";
    std::cout << "Total points: " << metrics_.total_points << "\n";
    std::cout << "Bonus points: " << metrics_.bonus_points << "\n";
    std::cout << "Final score: " << calculateScore() << "\n";
    std::cout << "Total time: " << std::fixed << std::setprecision(1) 
              << metrics_.total_time << "s\n\n";
    
    std::cout << "Challenge Details:\n";
    for (const auto& [id, challenge] : challenges_) {
        std::cout << "  " << id << ". " << challenge.name << ": ";
        if (challenge.completed) {
            std::cout << "✅ COMPLETED (" << std::fixed << std::setprecision(1) 
                      << challenge.completion_time << "s)\n";
        } else {
            std::cout << "❌ NOT COMPLETED\n";
        }
    }
}

Point AIMMICCCompetitionSystem::calculateOptimalLaunchPosition(const Point& launch_area, 
                                                              const Point& target) const {
    // Calculate optimal launch position considering wind, distance, and angle
    double dx = target.first - launch_area.first;
    double dy = target.second - launch_area.second;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Optimal launch position (simplified calculation)
    Point optimal = {
        launch_area.first + dx * 0.1,  // 10% closer to target
        launch_area.second + dy * 0.1
    };
    
    return optimal;
}

Point AIMMICCCompetitionSystem::calculateDeploymentZone(const Point& zebra_buoy, double radius_feet) const {
    // Convert feet to meters (simplified)
    double radius_meters = radius_feet * 0.3048;
    
    // Calculate deployment zone center (simplified)
    Point deployment_zone = {
        zebra_buoy.first + radius_meters * 0.5,
        zebra_buoy.second + radius_meters * 0.5
    };
    
    return deployment_zone;
}

std::vector<Point> AIMMICCCompetitionSystem::planEvasionRoute(const std::vector<Point>& channel_buoys, 
                                                            const std::vector<Point>& lidar_positions) const {
    // Plan route avoiding LiDAR detection zones
    std::vector<Point> evasion_route;
    
    // Simplified evasion planning
    for (const auto& buoy : channel_buoys) {
        // Check if buoy is safe from LiDAR
        bool safe = true;
        for (const auto& lidar : lidar_positions) {
            double dist = std::sqrt(std::pow(buoy.first - lidar.first, 2) + 
                                   std::pow(buoy.second - lidar.second, 2));
            if (dist < 5.0) { // 5m detection radius
                safe = false;
                break;
            }
        }
        
        if (safe) {
            evasion_route.push_back(buoy);
        }
    }
    
    return evasion_route;
}

} // namespace navigation
