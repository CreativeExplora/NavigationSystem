/**
 * Advanced algorithms for autonomous navigation.
 *
 * This module implements competition-grade algorithms for optimal pathfinding,
 * spatial queries, and route optimization.
 *
 * Key Algorithms:
 * - A* Pathfinding: O(E log V) with priority queue
 * - TSP Dynamic Programming: O(n^2 * 2^n) with bitmask DP
 * - Divide-and-Conquer Convex Hull: O(n log n) with KD-Tree integration
 * - KD-Tree Spatial Queries: O(sqrt(n)) average nearest neighbor
 * - Dijkstra's Algorithm: O((V + E) log V)
 * - Segment Tree Range Queries: O(log n) query/update
 * - Computational Geometry: Cross products, tangent lines, point-in-polygon
 */
#ifndef NAVIGATION_ADVANCED_ALGORITHMS_HPP
#define NAVIGATION_ADVANCED_ALGORITHMS_HPP

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <tuple>
#include <cmath>
#include <limits>
#include <functional>
#include <algorithm>
#include <optional>
#include <memory>

namespace navigation {

// ============================================================================
// TYPES AND STRUCTURES
// ============================================================================

using Point = std::pair<double, double>;
using Path = std::vector<Point>;

// Hash function for Point to use in unordered containers
struct PointHash {
    std::size_t operator()(const Point& p) const {
        std::size_t h1 = std::hash<double>{}(p.first);
        std::size_t h2 = std::hash<double>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

// ============================================================================
// PRIORITY QUEUE & A* PATHFINDING
// ============================================================================

struct PriorityNode {
    double priority;
    Point position;
    double g_cost;
    std::optional<Point> parent;

    PriorityNode(double p, Point pos, double g, std::optional<Point> par = std::nullopt)
        : priority(p), position(pos), g_cost(g), parent(par) {}

    bool operator>(const PriorityNode& other) const {
        return priority > other.priority;
    }
};

/**
 * Advanced pathfinding using multiple platinum-level algorithms:
 * - RRT* (Rapidly-exploring Random Tree*) - O(n log n)
 * - Visibility Graph + Dijkstra - O(n^2 log n)
 * - Hierarchical A* - O(log h) where h is hierarchy depth
 */
class AdvancedPathfinder {
private:
    double grid_resolution_;
    std::vector<std::tuple<double, double, double>> obstacles_; // (x, y, radius)

    double heuristic(const Point& pos1, const Point& pos2) const;
    std::vector<Point> getNeighbors(const Point& pos) const;

public:
    explicit AdvancedPathfinder(double grid_resolution = 1.0);

    void addObstacle(double x, double y, double radius);
    bool isValidPosition(double x, double y) const;
    Path findPath(const Point& start, const Point& goal);
};

// ============================================================================
// DYNAMIC PROGRAMMING - TRAVELING SALESMAN PROBLEM
// ============================================================================

/**
 * Dynamic Programming solution for Traveling Salesman Problem.
 * Time Complexity: O(n^2 * 2^n)
 * Space Complexity: O(n * 2^n)
 */
class TSPOptimizer {
private:
    std::vector<Point> points_;
    int n_;
    std::vector<std::vector<double>> dist_;
    std::map<std::pair<int, int>, std::pair<double, int>> memo_;

    void computeDistances();
    std::pair<double, std::vector<int>> greedyTSP(int start_idx);

public:
    explicit TSPOptimizer(const std::vector<Point>& points);

    /**
     * Find optimal tour using bitmask DP.
     * Returns: (total_distance, optimal_order)
     */
    std::pair<double, std::vector<int>> solve(int start_idx = 0);
};

// ============================================================================
// COMPUTATIONAL GEOMETRY
// ============================================================================

class GeometryUtils {
public:
    /**
     * Calculate cross product of vectors OA and OB.
     */
    static double crossProduct(const Point& o, const Point& a, const Point& b);

    /**
     * Convex hull algorithm using upper/lower hull construction.
     * Time Complexity: O(n log n)
     */
    static std::vector<Point> convexHull(std::vector<Point> points);

    /**
     * Find intersection point of two line segments.
     * Returns: intersection point or nullopt if no intersection
     */
    static std::optional<Point> lineIntersection(const Point& p1, const Point& p2,
                                                  const Point& p3, const Point& p4);

    /**
     * Ray casting algorithm to check if point is inside polygon.
     * Time Complexity: O(n)
     */
    static bool pointInPolygon(const Point& point, const std::vector<Point>& polygon);

    /**
     * Find closest point on line segment AB to point P.
     */
    static Point closestPointOnSegment(const Point& p, const Point& a, const Point& b);

    /**
     * Merge two convex hulls divided by a line.
     * Time Complexity: O(n + m) where n, m are hull sizes
     */
    static std::vector<Point> mergeConvexHulls(const std::vector<Point>& left_hull,
                                                const std::vector<Point>& right_hull,
                                                int axis = 0);

private:
    static std::vector<Point> ensureCCW(const std::vector<Point>& hull);
    static std::pair<int, int> findUpperTangent(const std::vector<Point>& left_hull,
                                                  const std::vector<Point>& right_hull);
    static std::pair<int, int> findLowerTangent(const std::vector<Point>& left_hull,
                                                  const std::vector<Point>& right_hull);
};

// ============================================================================
// SPATIAL DATA STRUCTURES
// ============================================================================

struct KDNode {
    Point point;
    std::shared_ptr<void> data; // Generic data storage
    std::shared_ptr<KDNode> left;
    std::shared_ptr<KDNode> right;

    KDNode(Point p, std::shared_ptr<void> d = nullptr)
        : point(p), data(d), left(nullptr), right(nullptr) {}

    bool isLeaf() const {
        return left == nullptr && right == nullptr;
    }
};

/**
 * 2D K-D Tree for efficient spatial queries.
 * Build Time: O(n log n)
 * Query Time: O(sqrt(n)) average case
 */
class KDTree {
private:
    std::shared_ptr<KDNode> root_;

    std::shared_ptr<KDNode> build(std::vector<std::pair<Point, std::shared_ptr<void>>>& points,
                                   int depth);
    void searchNearest(std::shared_ptr<KDNode> node, const Point& target,
                      int depth, Point& best_point,
                      std::shared_ptr<void>& best_data, double& best_dist) const;
    void searchRange(std::shared_ptr<KDNode> node, double min_x, double max_x,
                    double min_y, double max_y, int depth,
                    std::vector<std::pair<Point, std::shared_ptr<void>>>& result) const;

    // Divide-and-Conquer Convex Hull from KD-Tree
    std::vector<Point> convexHullFromKDTree(std::shared_ptr<KDNode> node, int depth) const;

public:
    explicit KDTree(std::vector<std::pair<Point, std::shared_ptr<void>>> points);

    /**
     * Find nearest neighbor to target point.
     * Returns: (point, data, distance)
     */
    std::tuple<Point, std::shared_ptr<void>, double> nearestNeighbor(const Point& target) const;

    /**
     * Find all points within rectangular range.
     */
    std::vector<std::pair<Point, std::shared_ptr<void>>> rangeSearch(
        double min_x, double max_x, double min_y, double max_y) const;

    /**
     * Compute convex hull using divide-and-conquer on KD-Tree.
     * This is the USACO Platinum-level algorithm that leverages
     * the KD-Tree's spatial partitioning.
     * Time: O(n log n)
     */
    std::vector<Point> computeConvexHullDC() const;

    std::shared_ptr<KDNode> getRoot() const { return root_; }
};

// ============================================================================
// SEGMENT TREE FOR RANGE QUERIES
// ============================================================================

/**
 * Segment tree for efficient range minimum queries.
 * Build Time: O(n)
 * Query Time: O(log n)
 * Update Time: O(log n)
 */
class SegmentTree {
private:
    int n_;
    std::vector<double> tree_;
    std::vector<double> arr_;

    void buildTree(int node, int start, int end);
    double queryRange(int node, int start, int end, int left, int right) const;
    void updateValue(int node, int start, int end, int idx, double value);

public:
    explicit SegmentTree(const std::vector<double>& arr);

    double query(int left, int right) const;
    void update(int idx, double value);
};

// ============================================================================
// BINARY SEARCH OPTIMIZATION
// ============================================================================

class BinarySearchOptimizer {
public:
    /**
     * Ternary search for finding optimal parameter value.
     * Time Complexity: O(log n)
     */
    static double findOptimalValue(std::function<double(double)> func,
                                   double low, double high,
                                   double epsilon = 1e-6,
                                   bool minimize = true);
};

// ============================================================================
// DIJKSTRA'S ALGORITHM WITH GRAPH
// ============================================================================

/**
 * Graph representation for navigation with Dijkstra's algorithm.
 * Time Complexity: O((V + E) log V) with priority queue
 */
class NavigationGraph {
private:
    std::map<int, std::vector<std::pair<int, double>>> graph_;
    std::map<int, Point> positions_;

public:
    NavigationGraph() = default;

    void addNode(int node_id, double x, double y);
    void addEdge(int from_node, int to_node, double weight);

    /**
     * Find shortest path using Dijkstra's algorithm.
     * Returns: (distance, path)
     */
    std::pair<double, std::vector<int>> dijkstra(int start, int end) const;
};

// ============================================================================
// DISJOINT SET UNION (UNION-FIND)
// ============================================================================

/**
 * Union-Find data structure with path compression and union by rank.
 * Time Complexity: O(α(n)) ≈ O(1) amortized per operation
 */
class DisjointSetUnion {
private:
    std::vector<int> parent_;
    std::vector<int> rank_;

public:
    explicit DisjointSetUnion(int n);

    int find(int x);
    bool unionSets(int x, int y);
    bool connected(int x, int y);
};

} // namespace navigation

#endif // NAVIGATION_ADVANCED_ALGORITHMS_HPP
