#include "navigation/buoy_detection.hpp"
#include <algorithm>

namespace navigation {

// Initialize static color map
const std::map<int, std::string> BuoyDetector::BUOY_COLOR_MAP = {
    {0, "free"}, {10, "l"}, {20, "r"}, {30, "g"}, {40, "y"},
    {50, "b"}, {60, "m"}, {70, "c"}, {80, "k"}, {90, "w"}, {100, "u"}
};

void BuoyDetector::addBuoy(double x, double y, int color) {
    buoys_.emplace_back(x, y, color);
}

std::string BuoyDetector::getColorString(int color_code) const {
    auto it = BUOY_COLOR_MAP.find(color_code);
    if (it != BUOY_COLOR_MAP.end()) {
        return it->second;
    }
    return "unknown";
}

double BuoyDetector::euclideanDistanceSq(double x1, double y1, double x2, double y2) const {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return dx * dx + dy * dy;
}

bool BuoyDetector::alreadySeen(const Buoy& buoy, const std::vector<Buoy>& exclude_list,
                               double distance) const {
    double dist_sq = distance * distance;
    for (const auto& excluded_buoy : exclude_list) {
        if (euclideanDistanceSq(buoy.x, buoy.y, excluded_buoy.x, excluded_buoy.y) < dist_sq) {
            return true;
        }
    }
    return false;
}

std::optional<Buoy> BuoyDetector::findClosestBuoy(double x, double y,
                                                  const std::vector<std::string>& colors,
                                                  double out_radius,
                                                  const std::vector<Buoy>* exclude) const {
    double min_distance = std::numeric_limits<double>::infinity();
    std::optional<Buoy> closest_buoy = std::nullopt;

    double out_radius_sq = out_radius * out_radius;

    for (const auto& buoy : buoys_) {
        std::string color_str = getColorString(buoy.color);

        // Check if color matches
        if (std::find(colors.begin(), colors.end(), color_str) != colors.end()) {
            double distance = euclideanDistanceSq(x, y, buoy.x, buoy.y);

            if (out_radius_sq < distance && distance < min_distance) {
                if (!exclude || !alreadySeen(buoy, *exclude)) {
                    min_distance = distance;
                    closest_buoy = buoy;
                }
            }
        }
    }

    return closest_buoy;
}

} // namespace navigation
