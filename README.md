# 🏆 AIMM-ICC Competition Navigation System

A competition-grade autonomous navigation system designed for the AIMM-ICC (Autonomous Intelligent Maritime Mission - International Competition), featuring advanced algorithms for maritime vehicle control and pathfinding, implemented in C++.

## 🚀 Features

### Advanced Algorithms
- **KD-Tree + D&C Convex Hull** - O(n log n) spatial partitioning with divide-and-conquer
- **RRT* Pathfinding** - O(n log n) probabilistic pathfinding using KD-Tree queries
- **TSP Dynamic Programming** - O(n² × 2ⁿ) optimal route optimization

### AIMM-ICC Competition Capabilities
- 🎯 **All 8 Challenges Supported** - Gate Navigation, Dodge Slalom, Evade, Identify, Deploy, Launch, Recover, Receive
- 🗺️ **Dynamic Obstacle Handling** - Real-time adaptation to moving buoys (~1m radius)
- 🛡️ **GPS-Agnostic Navigation** - Bonus points for sensor-only operation
- 📊 **Real-Time Scoring** - Live competition metrics and point tracking
- 🎮 **Multiple Modes** - Demo, Competition Run, Challenge Testing

## 🏗️ Architecture

```
Controls/
├── CMakeLists.txt             # CMake build configuration
├── src/                      # C++ source files
│   ├── main.cpp              # AIMM-ICC competition entry point
│   └── navigation/           # Core navigation system
│       ├── advanced_algorithms.cpp    # TSP, KD-Tree, geometry utils
│       ├── buoy_detection.cpp         # Buoy detection & classification
│       ├── config.cpp                 # Configuration management
│       ├── dynamic_kdtree.cpp         # Dynamic spatial data structure
│       ├── dynamic_navigation.cpp      # Real-time navigation system
│       └── aimm_icc_competition.cpp   # Competition challenge implementations
├── include/                  # C++ header files
│   └── navigation/           # Navigation headers
│       ├── advanced_algorithms.hpp
│       ├── buoy_detection.hpp
│       ├── config.hpp
│       ├── exceptions.hpp
│       ├── dynamic_kdtree.hpp
│       ├── dynamic_navigation.hpp
│       └── aimm_icc_competition.hpp
├── algorithms/               # Documentation and visualizations
│   └── convex_hull/          # Convex hull documentation
│       ├── CONVEX_HULL_DC.md # Documentation
│       ├── convex_hull_visualization.png
│       ├── kdtree_convex_hull_steps.png
│       └── tangent_finding_algorithm.png
├── AIMM_ICC_COMPLIANCE.md    # Competition requirements analysis
├── DYNAMIC_NAVIGATION.md     # Dynamic system documentation
└── aimm_icc_navigation       # Competition executable
```

## 🚀 Quick Start

### Build the System
```bash
mkdir build && cd build
cmake ..
make
```

### Run the System
```bash
./aimm_icc_navigation
```

Select mode:
- **1. Dynamic Navigation Demo** - Test dynamic obstacle handling
- **2. AIMM-ICC Competition Run** - Full 1-hour competition simulation
- **3. Challenge Testing** - Test individual challenges

Select difficulty level (Demo mode):
- **Easy** (4 buoys)
- **Medium** (8 buoys) 
- **Hard** (12 buoys)
- **Extreme** (22 buoys)

## 🧮 Algorithm Details

### Dynamic KD-Tree Architecture
- **Dynamic KD-Tree** with incremental updates (rebuilds when 20% of points change)
- **Real-time adaptation** to moving obstacles (~1m radius movement)
- **O(n log n)** construction, **O(√n)** query time, **O(log n)** amortized updates
- **Memory efficient** - smart pointer management with soft deletion

### KD-Tree + D&C Convex Hull
- Leverages KD-Tree's spatial partitioning structure
- Divide-and-conquer with tangent merging
- **Dynamic updates** as obstacles move with ocean currents
- Computes safe navigation zones efficiently

### RRT* Pathfinding
- Uses KD-Tree for nearest neighbor queries
- **Real-time collision avoidance** with continuous replanning
- **GPS-agnostic operation** for competition bonus points
- Probabilistically complete with optimal rewiring

### TSP Dynamic Programming
- **Optimal route optimization** for multiple waypoints
- **Adapts to moving obstacles** during navigation
- Uses the same dynamic point set from KD-Tree
- Bitmask DP for exponential state space

## 🎯 AIMM-ICC Competition Challenges

### Challenge #1: Gate Navigation ✅
- Navigate through 2-buoy gate (Lime Green + Red)
- **RRT* pathfinding** with collision avoidance
- **Zero contact bonus** points available

### Challenge #2: Dodge Slalom ✅
- Navigate buoys slalom course
- **TSP-DP optimization** for optimal route
- **Fastest time bonus** vs all teams

### Challenge #3: Evade
- Evade LiDAR detection along path
- **Convex hull computation** for safe zones
- **Dynamic adaptation** to detection zones

### Challenge #4: Identify
- Navigate to specific-colored buoy
- **Color-based detection** and navigation
- **Random color bonus** points

### Challenge #5: Deploy
- Navigate to "Zebra" buoy (black/white spiral)
- **Precision navigation** within 6' deployment zone
- **Sensor deployment** capabilities

### Challenge #6: Launch
- Navigate to Black Buoy (launch area)
- **Optimal positioning** for projectile accuracy
- **Aerial drone support** (launch/land from LPV)

### Challenge #7: Recover
- Find & retrieve floating object
- **Object detection** and retrieval planning
- **Damage prevention** algorithms

### Challenge #8: Receive
- Communicate with deployed sensor
- **Wireless communication** integration
- **Return navigation** to dock

## 🛠️ Configuration

The system uses `config.cpp` and `config.hpp` for:
- **Competition parameters** (safety radius, navigation speed, update rates)
- **Vehicle parameters** (boat specifications, sensor configurations)
- **Algorithm parameters** (KD-Tree rebuild thresholds, path planning intervals)
- **Safety margins** (collision avoidance distances, emergency protocols)

## 📈 Real-Time Performance

Dynamic system capabilities:
- **Obstacle tracking**: 25+ moving obstacles with realistic 1m radius movement
- **Path replanning**: Every 1 second for real-time adaptation
- **Update frequency**: 0.5 second intervals for responsive navigation
- **Safety radius**: 2m collision avoidance from moving buoys
- **Navigation speed**: 2 m/s realistic boat speed

## 🏆 Competition Advantages

### Bonus Point Opportunities
- **GPS-Agnostic**: Additional points per task (no GPS/external control)
- **Zero Collision**: Bonus for collision-free navigation
- **IR/RADAR Navigation**: Sensor-only navigation bonus
- **Fastest Time**: Speed bonuses for slalom and evade challenges
- **Random Color**: Bonus for judge-selected buoy color

### Technical Excellence
- **Dynamic KD-Tree**: Rebuilds only when 20% of points change
- **Real-Time Adaptation**: Handles moving obstacles efficiently
- **Advanced Algorithms**: RRT*, TSP-DP, D&C Convex Hull
- **Memory Efficiency**: Smart pointer management with RAII
- **Scalable Design**: Handles 4-22+ waypoints efficiently

## 📚 Documentation

- **[AIMM_ICC_COMPLIANCE.md](AIMM_ICC_COMPLIANCE.md)** - Detailed competition requirements analysis
- **[DYNAMIC_NAVIGATION.md](DYNAMIC_NAVIGATION.md)** - Dynamic system implementation details
- **[algorithms/convex_hull/CONVEX_HULL_DC.md](algorithms/convex_hull/CONVEX_HULL_DC.md)** - Algorithm documentation

## 📝 License

MIT - feel free to use and modify for your projects!
