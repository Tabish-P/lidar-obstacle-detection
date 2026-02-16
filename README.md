# LIDAR Obstacle Detection

A C++ project implementing a complete LIDAR-based obstacle detection pipeline for autonomous vehicles. This project simulates a highway environment with a LIDAR sensor, processes point cloud data, and detects obstacles using custom implementations of segmentation and clustering algorithms. Then we test the complete detection pipeline with real data from a self-driving car

![Detection Pipeline](/detection-pipeline.gif)

## Features

### Core Algorithm Implementations

- **RANSAC Segmentation**: 3D plane detection to separate road from obstacles
  - Custom 2D RANSAC implementation
  - PCL-based 3D RANSAC for production use

- **Clustering Algorithms**: Multiple approaches for grouping point cloud data
  - Custom KD-Tree implementation
  - Euclidean clustering from scratch
  - PCL's built-in clustering for efficiency

- **Point Cloud Processing**:
  - Voxel grid filtering for downsampling
  - Crop box filtering for region of interest
  - Ground plane removal
  - Obstacle isolation and clustering

- **Bounding Box Detection**: Axis-aligned bounding boxes for obstacle representation

### Data Processing Pipeline

```
Raw LIDAR Data → Filter → Segment (Remove Road) → Cluster → Bounding Box → Visualization
```

## Project Structure

```
├── src/
│   ├── environment.cpp          # Main simulation environment
│   ├── processPointClouds.h/cpp  # Core point cloud processing
│   ├── sensors/
│   │   ├── lidar.h              # LIDAR simulation
│   │   └── data/pcd/            # Sample PCD files
│   ├── render/
│   │   ├── render.h/cpp         # 3D visualization
│   │   └── box.h                # Bounding box representation
│   └── quiz/
│       ├── ransac/              # RANSAC implementation from scratch
│       └── cluster/             # KD-Tree and clustering implementation
├── build/                        # CMake build directory
└── CMakeLists.txt               # Build configuration
```

## Dependencies

- **PCL (Point Cloud Library)** >= 1.11: For point cloud processing and 3D visualization
- **C++17**: Modern C++ standard
- **CMake** >= 3.10: Build system
- **VTK**: Required by PCL for 3D visualization

### Installation (macOS)

```bash
# Install dependencies via Homebrew
brew install pcl cmake

# Or install PCL from source if needed
# https://pointclouds.org/documentation/tutorials/building_pcl.html
```

## Building and Running

### Build

```bash
cd /path/to/lidar-obstacle-detection
mkdir -p build && cd build
cmake ..
make
```

### Run Main Application

```bash
cd build
./environment
```

## Key Components

### 1. Lidar Sensor Simulation (`sensors/lidar.h`)
Simulates LIDAR by ray casting from a sensor position, detecting intersections with vehicle obstacles and adding Gaussian noise.

### 2. Point Cloud Processing (`processPointClouds.h/.cpp`)
Implements the main processing pipeline:
- Filtering and downsampling
- Plane segmentation (ground removal)
- Clustering of obstacles
- Bounding box generation

### 3. RANSAC Implementation
Custom implementation for robust plane detection in point clouds.

### 4. Clustering
Custom KD-Tree based Euclidean clustering from scratch, plus comparison with PCL's built-in clustering.

### 5. 3D Visualization (`render/`)
Real-time 3D rendering using PCL's visualization module showing:
- Highway scene and vehicles
- Point clouds
- Clusters with different colors
- Bounding boxes

## Algorithm Details

### RANSAC for 3D Plane Segmentation
Identifies inliers and outliers to segment the road plane from obstacle points, robust to noise and outliers.

### Euclidean Clustering with KD-Tree
Groups nearby points into clusters using a custom KD-Tree data structure for efficient nearest-neighbor queries.

### Voxel Grid Filtering
Reduces computational load by downsampling dense point clouds while preserving structure.

## Development Process

This project was developed progressively:

1. **Initial Setup** - Project structure and LIDAR simulation
2. **RANSAC Implementation** - 2D and 3D plane segmentation
3. **PCL Integration** - Leveraging PCL for clustering and segmentation
4. **Custom Clustering** - KD-Tree and Euclidean clustering from scratch
5. **Bounding Box Detection** - Adding obstacle boundaries
6. **Full Pipeline** - Complete streaming and detection system

# Changes Made

- Extended KD-Tree to 3D - Modified the KD-tree structure to handle 3D points by changing depth-based splitting from 2D (depth % 2) to 3D (depth % 3), including 3D distance calculations

- Added Custom RANSAC Plane Segmentation - Implemented 3D plane segmentation that randomly samples 3 points, fits planes using the equation Ax + By + Cz + D = 0, and finds the plane with maximum inliers

- Added Custom KD-Tree Clustering - Implemented Euclidean clustering using the extended 3D KD-tree for efficient neighbor searches with depth-first traversal

- Updated environment.cpp - Replaced all PCL algorithm calls for cityBlock

## Learning Outcomes

This project demonstrates:
- Point cloud processing and manipulation
- 3D geometry and computational geometry algorithms (RANSAC, KD-Trees)
- Real-time data processing pipelines
- 3D visualization techniques
- C++ template programming
- CMake build system
- Autonomous vehicle perception fundamentals

## Future Improvements

### Object Tracking

The current system detects obstacles in individual frames. Future implementations will add multi-frame tracking:

## License

This is an educational project developed as part of the Udacity Sensor Fusion Nanodegree.

## Author

Tabish Punjani & Udacity
