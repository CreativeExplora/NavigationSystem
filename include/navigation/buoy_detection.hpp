/**
 * Buoy detection and management module for autonomous navigation.
 */
#ifndef NAVIGATION_BUOY_DETECTION_HPP
#define NAVIGATION_BUOY_DETECTION_HPP

#include <vector>
#include <string>
#include <map>
#include <optional>
#include <memory>
#include <sstream>
#include <cmath>
#include <limits>

namespace navigation {

/**
 * Represents a detected buoy with position and color information.
 */
struct Buoy {
    double x;
    double y;
    int color;

    Buoy(double x_pos, double y_pos, int col) : x(x_pos), y(y_pos), color(col) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "Buoy(x=" << x << ", y=" << y << ", color=" << color << ")";
        return oss.str();
    }
};

/**
 * Handles buoy detection and color mapping.
 */
class BuoyDetector {
private:
    // Color mapping from numeric codes to string representations
    static const std::map<int, std::string> BUOY_COLOR_MAP;

    std::vector<Buoy> buoys_;

    /**
     * Calculate squared Euclidean distance between two points.
     */
    double euclideanDistanceSq(double x1, double y1, double x2, double y2) const;

    /**
     * Check if a buoy is already in the exclude list within given distance.
     */
    bool alreadySeen(const Buoy& buoy, const std::vector<Buoy>& exclude_list,
                     double distance = 2.0) const;

public:
    BuoyDetector() = default;

    /**
     * Add a buoy to the detection list.
     */
    void addBuoy(double x, double y, int color);

    /**
     * Get all buoys.
     */
    const std::vector<Buoy>& getBuoys() const { return buoys_; }

    /**
     * Convert numeric color code to string representation.
     */
    std::string getColorString(int color_code) const;

    /**
     * Find the closest buoy of specified colors within a given radius.
     *
     * Args:
     *     x, y: Current position
     *     colors: List of color strings to search for
     *     out_radius: Minimum distance to exclude (meters)
     *     exclude: List of buoys to exclude from search
     *
     * Returns:
     *     Closest matching buoy or nullopt if not found
     */
    std::optional<Buoy> findClosestBuoy(double x, double y,
                                       const std::vector<std::string>& colors,
                                       double out_radius = 0.0,
                                       const std::vector<Buoy>* exclude = nullptr) const;
};

} // namespace navigation

#endif // NAVIGATION_BUOY_DETECTION_HPP
