#pragma once

#include "navigation/dynamic_navigation.hpp"
#include <map>
#include <string>

namespace navigation {

/**
 * AIMM-ICC Competition-specific navigation system.
 * Handles all 8 competition challenges with optimized algorithms.
 */
class AIMMICCCompetitionSystem {
private:
    std::unique_ptr<DynamicNavigationSystem> nav_system_;
    
    // Competition-specific parameters
    struct CompetitionConfig {
        double safety_radius = 2.0;        // meters
        double navigation_speed = 2.0;     // m/s
        double update_rate = 0.5;          // seconds
        double replan_interval = 1.0;      // seconds
        bool gps_agnostic_mode = true;      // Bonus points
        bool zero_collision_mode = true;    // Bonus points
    } comp_config_;
    
    // Challenge-specific data
    struct ChallengeData {
        std::string name;
        int points_base;
        int points_bonus;
        bool completed;
        double completion_time;
    };
    
    std::map<int, ChallengeData> challenges_;
    
    // Competition metrics
    struct CompetitionMetrics {
        double total_time = 0.0;
        int challenges_completed = 0;
        int total_points = 0;
        int bonus_points = 0;
        bool gps_agnostic_bonus = false;
        bool zero_collision_bonus = false;
    } metrics_;
    
public:
    AIMMICCCompetitionSystem();
    
    // Challenge implementations
    bool challenge1_GateNavigation(const Point& dock_pos, const Point& lime_green_buoy, const Point& red_buoy);
    bool challenge2_DodgeSlalom(const std::vector<Point>& slalom_buoys);
    bool challenge3_Evade(const std::vector<Point>& channel_buoys, const std::vector<Point>& lidar_positions);
    bool challenge4_Identify(const Point& target_buoy, int target_color);
    bool challenge5_Deploy(const Point& zebra_buoy, const Point& deployment_position);
    bool challenge6_Launch(const Point& black_buoy, const Point& target_position);
    bool challenge7_Recover(const Point& floating_object);
    bool challenge8_Receive(const Point& return_buoy);
    
    // Competition control
    void startCompetition();
    void runCompetition(double time_limit = 3600.0); // 1 hour default
    void stopCompetition();
    
    // Scoring and metrics
    int calculateScore() const;
    CompetitionMetrics getMetrics() const;
    void printCompetitionStatus() const;
    
    // Bonus point opportunities
    bool enableGPSAgnosticMode();
    bool enableZeroCollisionMode();
    bool enableIRRadarNavigation();
    
    // Competition-specific navigation
    Point findBuoyByColor(int color) const;
    Point calculateOptimalLaunchPosition(const Point& launch_area, const Point& target) const;
    Point calculateDeploymentZone(const Point& zebra_buoy, double radius_feet = 6.0) const;
    std::vector<Point> planSlalomRoute(const std::vector<Point>& buoys) const;
    std::vector<Point> planEvasionRoute(const std::vector<Point>& channel_buoys, 
                                       const std::vector<Point>& lidar_positions) const;
    
    // Real-time competition monitoring
    void updateCompetitionMetrics(double dt);
    void logChallengeCompletion(int challenge_id, double time_taken, int points_earned);
    void printRealTimeStatus() const;
};

} // namespace navigation
