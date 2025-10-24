/**
 * Main entry point for the AIMM-ICC Competition Navigation System (C++ version).
 */
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <iomanip>

#include "navigation/config.hpp"
#include "navigation/buoy_detection.hpp"
#include "navigation/advanced_algorithms.hpp"
#include "navigation/exceptions.hpp"
#include "navigation/dynamic_navigation.hpp"
#include "navigation/aimm_icc_competition.hpp"

using namespace navigation;

/**
 * Dynamic navigation system demonstration with moving obstacles.
 * Shows how the shared KD-Tree handles real-time updates.
 */
class DynamicNavigationDemo {
private:
    std::unique_ptr<DynamicNavigationSystem> nav_system_;

    void demonstrateDynamicAlgorithms() {
        std::cout << "\n🏆 DEMONSTRATING DYNAMIC ADVANCED ALGORITHMS\n";
        std::cout << std::string(60, '=') << "\n";
        std::cout << "🌊 Environment: Moving obstacles + Static buoys\n";
        std::cout << "🔄 Real-time updates: KD-Tree rebuilds as needed\n";
        std::cout << "⚡ Continuous replanning: Every 1 second\n\n";

        // ============================================================
        // [1/3] DYNAMIC D&C CONVEX HULL - Updates with moving obstacles
        // ============================================================
        std::cout << "[1/3] Dynamic D&C Convex Hull...\n";
        auto hull = nav_system_->computeDynamicConvexHull();
        std::cout << "✓ Hull: " << hull.size() << " vertices (updates with obstacles)\n";
        std::cout << "  ✓ KD-Tree spatial partitioning adapts to movement\n";
        std::cout << "  ✓ Divide-and-conquer + tangent merging\n\n";

        // ============================================================
        // [2/3] DYNAMIC RRT* PATHFINDING - Real-time collision avoidance
        // ============================================================
        std::cout << "[2/3] Dynamic RRT* pathfinding...\n";
        Point start = {40.0, 5.0};
        Point goal = {50.0, 60.0};
        
        nav_system_->startNavigation(start, goal);
        std::cout << "✓ Navigation started: (" << start.first << ", " << start.second 
                  << ") → (" << goal.first << ", " << goal.second << ")\n";
        std::cout << "  ✓ Uses KD-Tree for real-time collision detection\n";
        std::cout << "  ✓ Continuous path replanning\n\n";

        // ============================================================
        // [3/3] DYNAMIC TSP-DP - Adapts to moving waypoints
        // ============================================================
        std::cout << "[3/3] Dynamic TSP-DP optimization...\n";
        auto [tsp_dist, order] = nav_system_->solveDynamicTSP(start);
        std::cout << "✓ TSP: " << tsp_dist << "m total (adapts to moving obstacles)\n";
        std::cout << "✓ Order: ";
        for (size_t i = 0; i < std::min(order.size(), size_t(5)); i++) {
            std::cout << order[i];
            if (i < std::min(order.size(), size_t(5)) - 1) std::cout << " → ";
        }
        if (order.size() > 5) std::cout << " → ...";
        std::cout << "\n";
        std::cout << "  ✓ DP on SAME dynamic point set from KD-Tree\n\n";

        // ============================================================
        // RUN DYNAMIC SIMULATION
        // ============================================================
        std::cout << "🌊 Running dynamic simulation...\n";
        nav_system_->runSimulation(10.0, 0.5);  // 10 seconds, 0.5s updates

        std::cout << "\n📊 DYNAMIC EFFICIENCY SUMMARY\n";
        std::cout << "   🌳 Dynamic KD-Tree: Rebuilds only when needed\n";
        std::cout << "   1️⃣  D&C Hull: Adapts to moving obstacles\n";
        std::cout << "   2️⃣  RRT*: Real-time collision avoidance\n";
        std::cout << "   3️⃣  TSP-DP: Updates with environment changes\n";
        std::cout << "\n   💡 One tree, three algorithms, dynamic updates!\n";
    }

public:
    DynamicNavigationDemo() {
        nav_system_ = std::make_unique<DynamicNavigationSystem>();
    }

    bool run(const std::string& difficulty) {
        std::cout << "\n🌊 Initializing dynamic environment...\n";
        nav_system_->initializeEnvironment(difficulty);
        
        demonstrateDynamicAlgorithms();

        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "🎉 DYNAMIC ADVANCED NAVIGATION COMPLETE 🎉\n";
        std::cout << std::string(70, '=') << "\n";

        return true;
    }
};

int main(int argc, char* argv[]) {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "🏆 AIMM-ICC COMPETITION NAVIGATION SYSTEM 🏆\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << "\nCompetition-grade algorithms:\n";
    std::cout << "  • KD-Tree + D&C Convex Hull - O(n log n)\n";
    std::cout << "  • RRT* Pathfinding - O(n log n)\n";
    std::cout << "  • TSP Dynamic Programming - O(n² × 2ⁿ)\n";
    std::cout << "\nAIMM-ICC Competition Ready:\n";
    std::cout << "  • All 8 challenges supported\n";
    std::cout << "  • GPS-agnostic navigation\n";
    std::cout << "  • Real-time obstacle avoidance\n";
    std::cout << "\n" << std::string(70, '=') << "\n";

    // Choose mode
    std::cout << "\nSelect mode:\n";
    std::cout << "  1. Dynamic Navigation Demo\n";
    std::cout << "  2. AIMM-ICC Competition Run\n";
    std::cout << "  3. Challenge Testing\n";

    std::string choice;
    std::cout << "\nChoice (1-3, default=1): ";
    std::getline(std::cin, choice);

    if (choice.empty()) choice = "1";

    try {
        if (choice == "2") {
            // AIMM-ICC Competition Mode
            AIMMICCCompetitionSystem competition_system;
            competition_system.runCompetition(3600.0); // 1 hour
            return 0;
        } else if (choice == "3") {
            // Challenge Testing Mode
            std::cout << "\n🧪 Challenge Testing Mode\n";
            std::cout << "Testing individual AIMM-ICC challenges...\n\n";
            
            AIMMICCCompetitionSystem test_system;
            
            // Test Challenge 1: Gate Navigation
            Point dock = {40.0, 5.0};
            Point lime_green = {42.0, 10.0};
            Point red_buoy = {48.0, 10.0};
            test_system.challenge1_GateNavigation(dock, lime_green, red_buoy);
            
            test_system.printCompetitionStatus();
            return 0;
        } else {
            // Dynamic Navigation Demo Mode
            std::cout << "\nSelect difficulty:\n";
            std::cout << "  1. Easy (4 buoys)\n";
            std::cout << "  2. Medium (8 buoys)\n";
            std::cout << "  3. Hard (12 buoys)\n";
            std::cout << "  4. Extreme (22 buoys)\n";

            std::string difficulty_choice;
            std::cout << "\nChoice (1-4, default=2): ";
            std::getline(std::cin, difficulty_choice);

            if (difficulty_choice.empty()) difficulty_choice = "2";

            std::string difficulty;
            if (difficulty_choice == "1") difficulty = "easy";
            else if (difficulty_choice == "2") difficulty = "medium";
            else if (difficulty_choice == "3") difficulty = "hard";
            else if (difficulty_choice == "4") difficulty = "extreme";
            else difficulty = "medium";

            DynamicNavigationDemo nav_demo;
            bool success = nav_demo.run(difficulty);
            return success ? 0 : 1;
        }
    } catch (const NavigationError& e) {
        std::cerr << "Navigation Error: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
