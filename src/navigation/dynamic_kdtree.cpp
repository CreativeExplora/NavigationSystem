#include "navigation/dynamic_kdtree.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

namespace navigation {

DynamicKDTree::DynamicKDTree(double rebuild_threshold)
    : root_(nullptr), max_depth_(0), rebuild_threshold_(rebuild_threshold), 
      operations_since_rebuild_(0) {}

void DynamicKDTree::insert(const Point& point, std::shared_ptr<void> data) {
    if (point_map_.find(point) != point_map_.end()) {
        // Point already exists, update data
        point_map_[point]->data = data;
        return;
    }
    
    root_ = insertNode(root_, point, data, 0);
    point_map_[point] = root_;  // Simplified - in practice, find the actual node
    all_points_.emplace_back(point, data);
    operations_since_rebuild_++;
    
    if (shouldRebuild()) {
        rebuildTree();
    }
}

std::shared_ptr<DynamicKDTree::KDNode> 
DynamicKDTree::insertNode(std::shared_ptr<KDNode> node, const Point& point, 
                         std::shared_ptr<void> data, int depth) {
    if (!node) {
        return std::make_shared<KDNode>(point, data, depth);
    }
    
    bool use_x = (depth % 2 == 0);
    bool go_left = use_x ? (point.first < node->point.first) : (point.second < node->point.second);
    
    if (go_left) {
        node->left = insertNode(node->left, point, data, depth + 1);
    } else {
        node->right = insertNode(node->right, point, data, depth + 1);
    }
    
    return node;
}

void DynamicKDTree::remove(const Point& point) {
    auto it = point_map_.find(point);
    if (it != point_map_.end()) {
        it->second->is_active = false;
        point_map_.erase(it);
        
        // Remove from all_points_
        all_points_.erase(
            std::remove_if(all_points_.begin(), all_points_.end(),
                [&point](const auto& p) { return p.first == point; }),
            all_points_.end()
        );
        
        operations_since_rebuild_++;
        
        if (shouldRebuild()) {
            rebuildTree();
        }
    }
}

void DynamicKDTree::update(const Point& old_point, const Point& new_point, std::shared_ptr<void> data) {
    remove(old_point);
    insert(new_point, data);
}

std::tuple<Point, std::shared_ptr<void>, double> 
DynamicKDTree::nearestNeighbor(const Point& target) const {
    if (!root_) {
        return {Point{0, 0}, nullptr, std::numeric_limits<double>::infinity()};
    }
    
    std::tuple<Point, std::shared_ptr<void>, double> best = {
        root_->point, root_->data, 
        std::sqrt(std::pow(target.first - root_->point.first, 2) + 
                 std::pow(target.second - root_->point.second, 2))
    };
    
    searchNearest(root_, target, best, 0);
    return best;
}

void DynamicKDTree::searchNearest(std::shared_ptr<KDNode> node, const Point& target, 
                                 std::tuple<Point, std::shared_ptr<void>, double>& best, 
                                 int depth) const {
    if (!node || !node->is_active) return;
    
    double dist = std::sqrt(std::pow(target.first - node->point.first, 2) + 
                           std::pow(target.second - node->point.second, 2));
    
    if (dist < std::get<2>(best)) {
        best = {node->point, node->data, dist};
    }
    
    bool use_x = (depth % 2 == 0);
    bool go_left = use_x ? (target.first < node->point.first) : (target.second < node->point.second);
    
    if (go_left) {
        searchNearest(node->left, target, best, depth + 1);
        // Check if we need to search the other side
        double axis_dist = use_x ? std::abs(target.first - node->point.first) : 
                                  std::abs(target.second - node->point.second);
        if (axis_dist < std::get<2>(best)) {
            searchNearest(node->right, target, best, depth + 1);
        }
    } else {
        searchNearest(node->right, target, best, depth + 1);
        double axis_dist = use_x ? std::abs(target.first - node->point.first) : 
                                  std::abs(target.second - node->point.second);
        if (axis_dist < std::get<2>(best)) {
            searchNearest(node->left, target, best, depth + 1);
        }
    }
}

std::vector<std::pair<Point, std::shared_ptr<void>>> 
DynamicKDTree::rangeSearch(double min_x, double max_x, double min_y, double max_y) const {
    std::vector<std::pair<Point, std::shared_ptr<void>>> result;
    if (root_) {
        searchRange(root_, min_x, max_x, min_y, max_y, result, 0);
    }
    return result;
}

void DynamicKDTree::searchRange(std::shared_ptr<KDNode> node, double min_x, double max_x, 
                               double min_y, double max_y, 
                               std::vector<std::pair<Point, std::shared_ptr<void>>>& result, 
                               int depth) const {
    if (!node || !node->is_active) return;
    
    const auto& [x, y] = node->point;
    if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
        result.emplace_back(node->point, node->data);
    }
    
    bool use_x = (depth % 2 == 0);
    
    if (use_x) {
        if (min_x <= node->point.first && node->left) {
            searchRange(node->left, min_x, max_x, min_y, max_y, result, depth + 1);
        }
        if (max_x >= node->point.first && node->right) {
            searchRange(node->right, min_x, max_x, min_y, max_y, result, depth + 1);
        }
    } else {
        if (min_y <= node->point.second && node->left) {
            searchRange(node->left, min_x, max_x, min_y, max_y, result, depth + 1);
        }
        if (max_y >= node->point.second && node->right) {
            searchRange(node->right, min_x, max_x, min_y, max_y, result, depth + 1);
        }
    }
}

std::vector<Point> DynamicKDTree::computeConvexHullDC() const {
    // Collect all active points
    std::vector<std::pair<Point, std::shared_ptr<void>>> active_points;
    collectActivePoints(root_, active_points);
    
    if (active_points.size() < 3) {
        return {};  // Need at least 3 points for convex hull
    }
    
    // Extract just the points
    std::vector<Point> points;
    for (const auto& [point, _] : active_points) {
        points.push_back(point);
    }
    
    // Use existing convex hull algorithm (upper/lower hull construction)
    std::sort(points.begin(), points.end());
    
    std::vector<Point> hull;
    for (const auto& point : points) {
        while (hull.size() >= 2) {
            const auto& a = hull[hull.size() - 2];
            const auto& b = hull[hull.size() - 1];
            double cross = (b.first - a.first) * (point.second - a.second) - 
                          (b.second - a.second) * (point.first - a.first);
            if (cross <= 0) hull.pop_back();
            else break;
        }
        hull.push_back(point);
    }
    
    size_t lower_hull_size = hull.size();
    for (int i = points.size() - 2; i >= 0; i--) {
        while (hull.size() > lower_hull_size) {
            const auto& a = hull[hull.size() - 2];
            const auto& b = hull[hull.size() - 1];
            double cross = (b.first - a.first) * (points[i].second - a.second) - 
                          (b.second - a.second) * (points[i].first - a.first);
            if (cross <= 0) hull.pop_back();
            else break;
        }
        hull.push_back(points[i]);
    }
    
    hull.pop_back();  // Remove duplicate
    return hull;
}

void DynamicKDTree::collectActivePoints(std::shared_ptr<KDNode> node, 
                                       std::vector<std::pair<Point, std::shared_ptr<void>>>& points) const {
    if (!node) return;
    
    if (node->is_active) {
        points.emplace_back(node->point, node->data);
    }
    
    collectActivePoints(node->left, points);
    collectActivePoints(node->right, points);
}

bool DynamicKDTree::shouldRebuild() const {
    if (all_points_.empty()) return false;
    return static_cast<double>(operations_since_rebuild_) / all_points_.size() > rebuild_threshold_;
}

void DynamicKDTree::rebuildTree() {
    std::vector<std::pair<Point, std::shared_ptr<void>>> active_points;
    collectActivePoints(root_, active_points);
    
    root_ = nullptr;
    point_map_.clear();
    all_points_ = active_points;
    operations_since_rebuild_ = 0;
    
    if (!active_points.empty()) {
        root_ = buildTree(active_points, 0, 0, active_points.size());
    }
}

std::shared_ptr<DynamicKDTree::KDNode> 
DynamicKDTree::buildTree(const std::vector<std::pair<Point, std::shared_ptr<void>>>& points, 
                        int depth, int start, int end) {
    if (start >= end) return nullptr;
    
    // Sort by current axis
    std::vector<std::pair<Point, std::shared_ptr<void>>> sorted_points(
        points.begin() + start, points.begin() + end);
    
    bool use_x = (depth % 2 == 0);
    std::sort(sorted_points.begin(), sorted_points.end(),
        [use_x](const auto& a, const auto& b) {
            return use_x ? (a.first.first < b.first.first) : (a.first.second < b.first.second);
        });
    
    int mid = sorted_points.size() / 2;
    auto node = std::make_shared<KDNode>(sorted_points[mid].first, sorted_points[mid].second, depth);
    
    // Build subtrees
    node->left = buildTree(sorted_points, depth + 1, 0, mid);
    node->right = buildTree(sorted_points, depth + 1, mid + 1, sorted_points.size());
    
    return node;
}

// Dynamic obstacle operations
void DynamicKDTree::addObstacle(double x, double y, double radius) {
    insert({x, y}, nullptr);
}

void DynamicKDTree::removeObstacle(double x, double y) {
    remove({x, y});
}

void DynamicKDTree::moveObstacle(double old_x, double old_y, double new_x, double new_y) {
    update({old_x, old_y}, {new_x, new_y}, nullptr);
}

// Status operations
size_t DynamicKDTree::size() const {
    return all_points_.size();
}

bool DynamicKDTree::isEmpty() const {
    return all_points_.empty();
}

void DynamicKDTree::optimize() {
    if (shouldRebuild()) {
        rebuildTree();
    }
}

void DynamicKDTree::clear() {
    root_ = nullptr;
    point_map_.clear();
    all_points_.clear();
    operations_since_rebuild_ = 0;
}

std::vector<Point> DynamicKDTree::getAllPoints() const {
    std::vector<Point> points;
    for (const auto& [point, _] : all_points_) {
        points.push_back(point);
    }
    return points;
}

} // namespace navigation
