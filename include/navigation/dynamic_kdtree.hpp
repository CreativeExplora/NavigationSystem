#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <chrono>

namespace navigation {

using Point = std::pair<double, double>;

/**
 * Dynamic KD-Tree that supports incremental updates for real-time navigation.
 * Handles moving obstacles, new detections, and continuous spatial queries.
 */
class DynamicKDTree {
private:
    struct KDNode {
        Point point;
        std::shared_ptr<void> data;
        std::shared_ptr<KDNode> left;
        std::shared_ptr<KDNode> right;
        int depth;
        bool is_active;  // For soft deletion
        
        KDNode(Point p, std::shared_ptr<void> d, int d_depth)
            : point(p), data(d), left(nullptr), right(nullptr), 
              depth(d_depth), is_active(true) {}
    };
    
    // Point hash for unordered_map
    struct PointHash {
        std::size_t operator()(const Point& p) const {
            return std::hash<double>()(p.first) ^ (std::hash<double>()(p.second) << 1);
        }
    };
    
    std::shared_ptr<KDNode> root_;
    std::unordered_map<Point, std::shared_ptr<KDNode>, PointHash> point_map_;
    std::vector<std::pair<Point, std::shared_ptr<void>>> all_points_;
    int max_depth_;
    double rebuild_threshold_;
    int operations_since_rebuild_;
    
    std::shared_ptr<KDNode> buildTree(const std::vector<std::pair<Point, std::shared_ptr<void>>>& points, 
                                     int depth, int start, int end);
    std::shared_ptr<KDNode> insertNode(std::shared_ptr<KDNode> node, const Point& point, 
                                     std::shared_ptr<void> data, int depth);
    std::shared_ptr<KDNode> removeNode(std::shared_ptr<KDNode> node, const Point& point, int depth);
    void collectActivePoints(std::shared_ptr<KDNode> node, 
                           std::vector<std::pair<Point, std::shared_ptr<void>>>& points) const;
    void searchNearest(std::shared_ptr<KDNode> node, const Point& target, 
                      std::tuple<Point, std::shared_ptr<void>, double>& best, int depth) const;
    void searchRange(std::shared_ptr<KDNode> node, double min_x, double max_x, 
                   double min_y, double max_y, std::vector<std::pair<Point, std::shared_ptr<void>>>& result, 
                   int depth) const;
    bool shouldRebuild() const;
    void rebuildTree();
    
public:
    explicit DynamicKDTree(double rebuild_threshold = 0.3);
    
    // Core operations
    void insert(const Point& point, std::shared_ptr<void> data = nullptr);
    void remove(const Point& point);
    void update(const Point& old_point, const Point& new_point, std::shared_ptr<void> data = nullptr);
    
    // Spatial queries
    std::tuple<Point, std::shared_ptr<void>, double> nearestNeighbor(const Point& target) const;
    std::vector<std::pair<Point, std::shared_ptr<void>>> rangeSearch(double min_x, double max_x, 
                                                                   double min_y, double max_y) const;
    
    // Convex hull operations
    std::vector<Point> computeConvexHullDC() const;
    
    // Dynamic operations
    void addObstacle(double x, double y, double radius = 1.0);
    void removeObstacle(double x, double y);
    void moveObstacle(double old_x, double old_y, double new_x, double new_y);
    
    // Status and maintenance
    size_t size() const;
    bool isEmpty() const;
    void optimize();  // Rebuild if needed
    void clear();
    
    // Debugging
    void printTree() const;
    std::vector<Point> getAllPoints() const;
};

} // namespace navigation
