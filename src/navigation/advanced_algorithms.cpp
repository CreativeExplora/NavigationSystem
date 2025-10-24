#include "navigation/advanced_algorithms.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <queue>

namespace navigation {

// ============================================================================
// ADVANCED PATHFINDER
// ============================================================================

AdvancedPathfinder::AdvancedPathfinder(double grid_resolution)
    : grid_resolution_(grid_resolution) {}

void AdvancedPathfinder::addObstacle(double x, double y, double radius) {
    obstacles_.emplace_back(x, y, radius);
}

bool AdvancedPathfinder::isValidPosition(double x, double y) const {
    for (const auto& [ox, oy, radius] : obstacles_) {
        double dist_sq = (x - ox) * (x - ox) + (y - oy) * (y - oy);
        if (dist_sq < (radius * 1.5) * (radius * 1.5)) {
            return false;
        }
    }
    return true;
}

double AdvancedPathfinder::heuristic(const Point& pos1, const Point& pos2) const {
    double dx = pos1.first - pos2.first;
    double dy = pos1.second - pos2.second;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Point> AdvancedPathfinder::getNeighbors(const Point& pos) const {
    std::vector<Point> neighbors;
    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    for (const auto& [dx, dy] : directions) {
        double nx = pos.first + dx * grid_resolution_;
        double ny = pos.second + dy * grid_resolution_;
        if (isValidPosition(nx, ny)) {
            neighbors.emplace_back(nx, ny);
        }
    }

    return neighbors;
}

Path AdvancedPathfinder::findPath(const Point& start, const Point& goal) {
    std::priority_queue<PriorityNode, std::vector<PriorityNode>,
                       std::greater<PriorityNode>> open_set;

    open_set.emplace(heuristic(start, goal), start, 0.0, std::nullopt);

    std::unordered_set<Point, PointHash> closed_set;
    std::unordered_map<Point, Point, PointHash> came_from;
    std::unordered_map<Point, double, PointHash> g_score;
    g_score[start] = 0.0;

    int max_iterations = 1000;
    int iterations = 0;

    while (!open_set.empty() && iterations < max_iterations) {
        iterations++;
        PriorityNode current = open_set.top();
        open_set.pop();

        if (heuristic(current.position, goal) < grid_resolution_) {
            // Reconstruct path
            Path path;
            Point pos = current.position;
            path.push_back(pos);

            while (came_from.find(pos) != came_from.end()) {
                pos = came_from[pos];
                path.push_back(pos);
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_set.find(current.position) != closed_set.end()) {
            continue;
        }

        closed_set.insert(current.position);

        for (const auto& neighbor : getNeighbors(current.position)) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            double tentative_g = current.g_cost + heuristic(current.position, neighbor);

            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current.position;
                g_score[neighbor] = tentative_g;
                double f_score = tentative_g + heuristic(neighbor, goal);
                open_set.emplace(f_score, neighbor, tentative_g, current.position);
            }
        }
    }

    std::cout << "⚠ A* failed after " << iterations << " iterations, using direct path\n";
    return {start, goal};
}

// ============================================================================
// TSP OPTIMIZER
// ============================================================================

TSPOptimizer::TSPOptimizer(const std::vector<Point>& points)
    : points_(points), n_(points.size()) {
    computeDistances();
}

void TSPOptimizer::computeDistances() {
    dist_.resize(n_, std::vector<double>(n_, 0.0));
    for (int i = 0; i < n_; i++) {
        for (int j = 0; j < n_; j++) {
            double dx = points_[i].first - points_[j].first;
            double dy = points_[i].second - points_[j].second;
            dist_[i][j] = std::sqrt(dx * dx + dy * dy);
        }
    }
}

std::pair<double, std::vector<int>> TSPOptimizer::solve(int start_idx) {
    if (n_ == 1) {
        return {0.0, {0}};
    }

    if (n_ > 20) {
        return greedyTSP(start_idx);
    }

    // DP with bitmask
    int all_visited = (1 << n_) - 1;
    memo_.clear();

    std::function<std::pair<double, int>(int, int)> dp =
        [&](int mask, int pos) -> std::pair<double, int> {
        if (mask == all_visited) {
            return {0.0, -1};
        }

        auto key = std::make_pair(mask, pos);
        if (memo_.find(key) != memo_.end()) {
            return memo_[key];
        }

        double best_dist = std::numeric_limits<double>::infinity();
        int best_next = -1;

        for (int next_pos = 0; next_pos < n_; next_pos++) {
            if (mask & (1 << next_pos)) {
                continue;
            }

            int new_mask = mask | (1 << next_pos);
            auto [future_dist, _] = dp(new_mask, next_pos);
            double total_dist = dist_[pos][next_pos] + future_dist;

            if (total_dist < best_dist) {
                best_dist = total_dist;
                best_next = next_pos;
            }
        }

        memo_[key] = {best_dist, best_next};
        return {best_dist, best_next};
    };

    // Reconstruct path
    int mask = 1 << start_idx;
    int pos = start_idx;
    std::vector<int> order = {start_idx};
    double total_dist = 0.0;

    while (true) {
        auto [dist, next_pos] = dp(mask, pos);
        if (next_pos == -1) {
            break;
        }
        total_dist += dist_[pos][next_pos];
        mask |= (1 << next_pos);
        pos = next_pos;
        order.push_back(next_pos);
    }

    return {total_dist, order};
}

std::pair<double, std::vector<int>> TSPOptimizer::greedyTSP(int start_idx) {
    std::vector<bool> visited(n_, false);
    std::vector<int> order = {start_idx};
    visited[start_idx] = true;
    double total_dist = 0.0;
    int current = start_idx;

    for (int i = 0; i < n_ - 1; i++) {
        double best_dist = std::numeric_limits<double>::infinity();
        int best_next = -1;

        for (int next_pos = 0; next_pos < n_; next_pos++) {
            if (!visited[next_pos]) {
                double d = dist_[current][next_pos];
                if (d < best_dist) {
                    best_dist = d;
                    best_next = next_pos;
                }
            }
        }

        if (best_next != -1) {
            total_dist += best_dist;
            visited[best_next] = true;
            order.push_back(best_next);
            current = best_next;
        }
    }

    return {total_dist, order};
}

// ============================================================================
// GEOMETRY UTILS
// ============================================================================

double GeometryUtils::crossProduct(const Point& o, const Point& a, const Point& b) {
    return (a.first - o.first) * (b.second - o.second) -
           (a.second - o.second) * (b.first - o.first);
}

std::vector<Point> GeometryUtils::convexHull(std::vector<Point> points) {
    if (points.size() < 3) {
        return points;
    }

    std::sort(points.begin(), points.end());

    // Build lower hull
    std::vector<Point> lower;
    for (const auto& p : points) {
        while (lower.size() >= 2 &&
               crossProduct(lower[lower.size()-2], lower[lower.size()-1], p) <= 0) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    // Build upper hull
    std::vector<Point> upper;
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        while (upper.size() >= 2 &&
               crossProduct(upper[upper.size()-2], upper[upper.size()-1], *it) <= 0) {
            upper.pop_back();
        }
        upper.push_back(*it);
    }

    // Remove last point of each half (duplicate)
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());

    return lower;
}

bool GeometryUtils::pointInPolygon(const Point& point, const std::vector<Point>& polygon) {
    double x = point.first;
    double y = point.second;
    int n = polygon.size();
    bool inside = false;

    int j = n - 1;
    for (int i = 0; i < n; i++) {
        double xi = polygon[i].first;
        double yi = polygon[i].second;
        double xj = polygon[j].first;
        double yj = polygon[j].second;

        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }

        j = i;
    }

    return inside;
}

// Stub implementations for more complex geometry functions
std::optional<Point> GeometryUtils::lineIntersection(const Point& p1, const Point& p2,
                                                      const Point& p3, const Point& p4) {
    // Implementation omitted for brevity
    return std::nullopt;
}

Point GeometryUtils::closestPointOnSegment(const Point& p, const Point& a, const Point& b) {
    double dx = b.first - a.first;
    double dy = b.second - a.second;

    if (dx == 0 && dy == 0) {
        return a;
    }

    double t = std::max(0.0, std::min(1.0,
        ((p.first - a.first) * dx + (p.second - a.second) * dy) / (dx * dx + dy * dy)));

    return {a.first + t * dx, a.second + t * dy};
}

std::vector<Point> GeometryUtils::mergeConvexHulls(const std::vector<Point>& left_hull,
                                                    const std::vector<Point>& right_hull,
                                                    int axis) {
    if (left_hull.empty()) return right_hull;
    if (right_hull.empty()) return left_hull;

    // For small hulls or horizontal splits, use simple convex hull
    if (left_hull.size() + right_hull.size() <= 4 || axis == 1) {
        std::vector<Point> combined = left_hull;
        combined.insert(combined.end(), right_hull.begin(), right_hull.end());
        return convexHull(combined);
    }

    // Ensure hulls are in counter-clockwise order
    std::vector<Point> left_ccw = ensureCCW(left_hull);
    std::vector<Point> right_ccw = ensureCCW(right_hull);

    // Find tangent points (for vertical split, axis=0)
    auto [upper_left_idx, upper_right_idx] = findUpperTangent(left_ccw, right_ccw);
    auto [lower_left_idx, lower_right_idx] = findLowerTangent(left_ccw, right_ccw);

    // Merge hulls using tangent points as bridges
    std::vector<Point> merged;

    // Add left hull from lower to upper
    int idx = lower_left_idx;
    while (true) {
        merged.push_back(left_ccw[idx]);
        if (idx == upper_left_idx) {
            break;
        }
        idx = (idx + 1) % left_ccw.size();
    }

    // Add right hull from upper to lower
    idx = upper_right_idx;
    while (true) {
        merged.push_back(right_ccw[idx]);
        if (idx == lower_right_idx) {
            break;
        }
        idx = (idx + 1) % right_ccw.size();
    }

    return merged;
}

std::vector<Point> GeometryUtils::ensureCCW(const std::vector<Point>& hull) {
    if (hull.size() < 3) return hull;

    double area = 0.0;
    for (size_t i = 0; i < hull.size(); i++) {
        size_t j = (i + 1) % hull.size();
        area += hull[i].first * hull[j].second;
        area -= hull[j].first * hull[i].second;
    }

    if (area < 0) {
        return std::vector<Point>(hull.rbegin(), hull.rend());
    }
    return hull;
}

std::pair<int, int> GeometryUtils::findUpperTangent(const std::vector<Point>& left_hull,
                                                     const std::vector<Point>& right_hull) {
    int n_left = left_hull.size();
    int n_right = right_hull.size();

    // Start from rightmost point of left hull and leftmost of right hull
    int left_idx = 0;
    int right_idx = 0;

    // Find rightmost point of left hull
    for (int i = 1; i < n_left; i++) {
        if (left_hull[i].first > left_hull[left_idx].first) {
            left_idx = i;
        }
    }

    // Find leftmost point of right hull
    for (int i = 1; i < n_right; i++) {
        if (right_hull[i].first < right_hull[right_idx].first) {
            right_idx = i;
        }
    }

    // Climb up both hulls until we find the upper tangent
    bool done = false;
    while (!done) {
        done = true;

        // Check if we should move up on left hull
        while (true) {
            int next_left = (left_idx + 1) % n_left;
            if (crossProduct(right_hull[right_idx], left_hull[left_idx], left_hull[next_left]) <= 0) {
                break;
            }
            left_idx = next_left;
            done = false;
        }

        // Check if we should move up on right hull
        while (true) {
            int prev_right = (right_idx - 1 + n_right) % n_right;
            if (crossProduct(left_hull[left_idx], right_hull[right_idx], right_hull[prev_right]) >= 0) {
                break;
            }
            right_idx = prev_right;
            done = false;
        }
    }

    return {left_idx, right_idx};
}

std::pair<int, int> GeometryUtils::findLowerTangent(const std::vector<Point>& left_hull,
                                                     const std::vector<Point>& right_hull) {
    int n_left = left_hull.size();
    int n_right = right_hull.size();

    // Start from rightmost point of left hull and leftmost of right hull
    int left_idx = 0;
    int right_idx = 0;

    // Find rightmost point of left hull
    for (int i = 1; i < n_left; i++) {
        if (left_hull[i].first > left_hull[left_idx].first) {
            left_idx = i;
        }
    }

    // Find leftmost point of right hull
    for (int i = 1; i < n_right; i++) {
        if (right_hull[i].first < right_hull[right_idx].first) {
            right_idx = i;
        }
    }

    // Climb down both hulls until we find the lower tangent
    bool done = false;
    while (!done) {
        done = true;

        // Check if we should move down on left hull
        while (true) {
            int prev_left = (left_idx - 1 + n_left) % n_left;
            if (crossProduct(right_hull[right_idx], left_hull[left_idx], left_hull[prev_left]) >= 0) {
                break;
            }
            left_idx = prev_left;
            done = false;
        }

        // Check if we should move down on right hull
        while (true) {
            int next_right = (right_idx + 1) % n_right;
            if (crossProduct(left_hull[left_idx], right_hull[right_idx], right_hull[next_right]) <= 0) {
                break;
            }
            right_idx = next_right;
            done = false;
        }
    }

    return {left_idx, right_idx};
}

// ============================================================================
// KD-TREE
// ============================================================================

KDTree::KDTree(std::vector<std::pair<Point, std::shared_ptr<void>>> points) {
    root_ = build(points, 0);
}

std::shared_ptr<KDNode> KDTree::build(
    std::vector<std::pair<Point, std::shared_ptr<void>>>& points, int depth) {

    if (points.empty()) {
        return nullptr;
    }

    int axis = depth % 2;
    std::sort(points.begin(), points.end(),
        [axis](const auto& a, const auto& b) {
            return (axis == 0 ? a.first.first : a.first.second) <
                   (axis == 0 ? b.first.first : b.first.second);
        });

    size_t median = points.size() / 2;
    auto node = std::make_shared<KDNode>(points[median].first, points[median].second);

    std::vector<std::pair<Point, std::shared_ptr<void>>> left_points(
        points.begin(), points.begin() + median);
    std::vector<std::pair<Point, std::shared_ptr<void>>> right_points(
        points.begin() + median + 1, points.end());

    node->left = build(left_points, depth + 1);
    node->right = build(right_points, depth + 1);

    return node;
}

std::tuple<Point, std::shared_ptr<void>, double>
KDTree::nearestNeighbor(const Point& target) const {
    Point best_point = {0, 0};
    std::shared_ptr<void> best_data = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();

    searchNearest(root_, target, 0, best_point, best_data, best_dist);

    return {best_point, best_data, best_dist};
}

void KDTree::searchNearest(std::shared_ptr<KDNode> node, const Point& target,
                          int depth, Point& best_point,
                          std::shared_ptr<void>& best_data, double& best_dist) const {
    if (!node) return;

    double dx = node->point.first - target.first;
    double dy = node->point.second - target.second;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < best_dist) {
        best_point = node->point;
        best_data = node->data;
        best_dist = dist;
    }

    int axis = depth % 2;
    double diff = (axis == 0 ? target.first - node->point.first :
                              target.second - node->point.second);

    if (diff < 0) {
        searchNearest(node->left, target, depth + 1, best_point, best_data, best_dist);
        if (std::abs(diff) < best_dist) {
            searchNearest(node->right, target, depth + 1, best_point, best_data, best_dist);
        }
    } else {
        searchNearest(node->right, target, depth + 1, best_point, best_data, best_dist);
        if (std::abs(diff) < best_dist) {
            searchNearest(node->left, target, depth + 1, best_point, best_data, best_dist);
        }
    }
}

std::vector<std::pair<Point, std::shared_ptr<void>>>
KDTree::rangeSearch(double min_x, double max_x, double min_y, double max_y) const {
    std::vector<std::pair<Point, std::shared_ptr<void>>> result;
    searchRange(root_, min_x, max_x, min_y, max_y, 0, result);
    return result;
}

void KDTree::searchRange(std::shared_ptr<KDNode> node, double min_x, double max_x,
                        double min_y, double max_y, int depth,
                        std::vector<std::pair<Point, std::shared_ptr<void>>>& result) const {
    if (!node) return;

    double x = node->point.first;
    double y = node->point.second;

    if (min_x <= x && x <= max_x && min_y <= y && y <= max_y) {
        result.emplace_back(node->point, node->data);
    }

    int axis = depth % 2;

    if (axis == 0) {
        if (min_x <= x) searchRange(node->left, min_x, max_x, min_y, max_y, depth + 1, result);
        if (x <= max_x) searchRange(node->right, min_x, max_x, min_y, max_y, depth + 1, result);
    } else {
        if (min_y <= y) searchRange(node->left, min_x, max_x, min_y, max_y, depth + 1, result);
        if (y <= max_y) searchRange(node->right, min_x, max_x, min_y, max_y, depth + 1, result);
    }
}

// Divide-and-Conquer Convex Hull from KD-Tree
std::vector<Point> KDTree::convexHullFromKDTree(std::shared_ptr<KDNode> node, int depth) const {
    if (!node) {
        return {};
    }

    // Leaf node - return single point
    if (node->isLeaf()) {
        return {node->point};
    }

    // Recursively compute hulls for left and right subtrees
    std::vector<Point> hull_left = convexHullFromKDTree(node->left, depth + 1);
    std::vector<Point> hull_right = convexHullFromKDTree(node->right, depth + 1);

    // Merge the two convex hulls
    int axis = depth % 2;  // 0 = vertical split (x-axis), 1 = horizontal split (y-axis)
    return GeometryUtils::mergeConvexHulls(hull_left, hull_right, axis);
}

std::vector<Point> KDTree::computeConvexHullDC() const {
    return convexHullFromKDTree(root_, 0);
}

// ============================================================================
// NAVIGATION GRAPH & DIJKSTRA
// ============================================================================

void NavigationGraph::addNode(int node_id, double x, double y) {
    positions_[node_id] = {x, y};
}

void NavigationGraph::addEdge(int from_node, int to_node, double weight) {
    graph_[from_node].emplace_back(to_node, weight);
}

std::pair<double, std::vector<int>> NavigationGraph::dijkstra(int start, int end) const {
    std::map<int, double> distances;
    std::map<int, int> parent;

    for (const auto& [node, _] : positions_) {
        distances[node] = std::numeric_limits<double>::infinity();
        parent[node] = -1;
    }
    distances[start] = 0;

    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;
    pq.emplace(0, start);

    std::set<int> visited;

    while (!pq.empty()) {
        auto [current_dist, current] = pq.top();
        pq.pop();

        if (visited.find(current) != visited.end()) {
            continue;
        }

        visited.insert(current);

        if (current == end) {
            break;
        }

        if (graph_.find(current) != graph_.end()) {
            for (const auto& [neighbor, weight] : graph_.at(current)) {
                double distance = current_dist + weight;

                if (distance < distances[neighbor]) {
                    distances[neighbor] = distance;
                    parent[neighbor] = current;
                    pq.emplace(distance, neighbor);
                }
            }
        }
    }

    // Reconstruct path
    if (distances[end] == std::numeric_limits<double>::infinity()) {
        return {std::numeric_limits<double>::infinity(), {}};
    }

    std::vector<int> path;
    int current = end;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }

    std::reverse(path.begin(), path.end());
    return {distances[end], path};
}

// ============================================================================
// DISJOINT SET UNION
// ============================================================================

DisjointSetUnion::DisjointSetUnion(int n) : parent_(n), rank_(n, 0) {
    for (int i = 0; i < n; i++) {
        parent_[i] = i;
    }
}

int DisjointSetUnion::find(int x) {
    if (parent_[x] != x) {
        parent_[x] = find(parent_[x]);
    }
    return parent_[x];
}

bool DisjointSetUnion::unionSets(int x, int y) {
    int root_x = find(x);
    int root_y = find(y);

    if (root_x == root_y) {
        return false;
    }

    if (rank_[root_x] < rank_[root_y]) {
        parent_[root_x] = root_y;
    } else if (rank_[root_x] > rank_[root_y]) {
        parent_[root_y] = root_x;
    } else {
        parent_[root_y] = root_x;
        rank_[root_x]++;
    }

    return true;
}

bool DisjointSetUnion::connected(int x, int y) {
    return find(x) == find(y);
}

// ============================================================================
// SEGMENT TREE (Stub)
// ============================================================================

SegmentTree::SegmentTree(const std::vector<double>& arr)
    : n_(arr.size()), arr_(arr), tree_(4 * n_, std::numeric_limits<double>::infinity()) {
    if (!arr.empty()) {
        buildTree(0, 0, n_ - 1);
    }
}

void SegmentTree::buildTree(int node, int start, int end) {
    if (start == end) {
        tree_[node] = arr_[start];
    } else {
        int mid = (start + end) / 2;
        buildTree(2 * node + 1, start, mid);
        buildTree(2 * node + 2, mid + 1, end);
        tree_[node] = std::min(tree_[2 * node + 1], tree_[2 * node + 2]);
    }
}

double SegmentTree::query(int left, int right) const {
    return queryRange(0, 0, n_ - 1, left, right);
}

double SegmentTree::queryRange(int node, int start, int end, int left, int right) const {
    if (left > end || right < start) {
        return std::numeric_limits<double>::infinity();
    }

    if (left <= start && end <= right) {
        return tree_[node];
    }

    int mid = (start + end) / 2;
    double left_min = queryRange(2 * node + 1, start, mid, left, right);
    double right_min = queryRange(2 * node + 2, mid + 1, end, left, right);

    return std::min(left_min, right_min);
}

void SegmentTree::update(int idx, double value) {
    arr_[idx] = value;
    updateValue(0, 0, n_ - 1, idx, value);
}

void SegmentTree::updateValue(int node, int start, int end, int idx, double value) {
    if (start == end) {
        tree_[node] = value;
    } else {
        int mid = (start + end) / 2;
        if (idx <= mid) {
            updateValue(2 * node + 1, start, mid, idx, value);
        } else {
            updateValue(2 * node + 2, mid + 1, end, idx, value);
        }
        tree_[node] = std::min(tree_[2 * node + 1], tree_[2 * node + 2]);
    }
}

// ============================================================================
// BINARY SEARCH OPTIMIZER
// ============================================================================

double BinarySearchOptimizer::findOptimalValue(std::function<double(double)> func,
                                               double low, double high,
                                               double epsilon, bool minimize) {
    while (high - low > epsilon) {
        double mid1 = low + (high - low) / 3;
        double mid2 = high - (high - low) / 3;

        double f1 = func(mid1);
        double f2 = func(mid2);

        if (minimize) {
            if (f1 > f2) {
                low = mid1;
            } else {
                high = mid2;
            }
        } else {
            if (f1 < f2) {
                low = mid1;
            } else {
                high = mid2;
            }
        }
    }

    return (low + high) / 2;
}

} // namespace navigation
