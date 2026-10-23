# Photo-Realistic Synthetic Data Generation & Intelligent Drone Coverage

This repository offers a complete pipeline for generating photo-realistic synthetic environments and intelligent drone coverage systems. It combines point clouds and orthophoto images to create realistic farm environments, supports animal insertion, and provides advanced waypoint generation algorithms for autonomous drone surveillance of livestock.

## Repository Structure

This project consists of four main sections:

### 1. Animal-Insertion  
Generates realistic coordinates to spawn animals within the simulated farm environment.

### 2. Map-Generation  
Creates synthetic maps of varying sizes by merging point clouds and orthophoto imagery.

### 3. ROS-Gazebo Simulation  
Complete ROS Noetic simulation environment with Hector quadrotor for autonomous drone operations.

### 4. Semantic-Segmentation
Generates semantic segmentation masks and bounding boxes for computer vision applications.

## Prerequisites

- Docker installed on your system
- Docker Compose installed on your system
- virtualenv installed on your system

## Getting Started

### 1. Clone the Repository

First, clone this repository to your local machine:

```bash
git clone https://github.com/MaVILab-UFV/2024-PIBIC-lucas-farm.git
cd 2024-PIBIC-lucas-farm.git/ros-gazebo-simulation
```

### 2. Build the Docker Environment
Navigate to the ROS-Gazebo simulation directory and build the Docker environment:

```bash
cd ros-gazebo-simulation
docker compose build base_farm_image
```

### 3. Generate Waypoints for Drone Coverage

The system provides multiple intelligent waypoint generation strategies:

#### Traditional Coverage (Zigzag Pattern)
```bash
docker compose up waypoint_generator
```

#### Animal-Targeted Coverage Strategies

**Naive Strategy** - Visits animals in sequence:
```bash
docker compose up animal_waypoint_generator_naive
```

**Optimized Strategy** - Uses TSP optimization for shortest path:
```bash
docker compose up animal_waypoint_generator_optimized
```

**Behavioral Strategy** - Herd-aware spiral approach to minimize animal disturbance:
```bash
docker compose up animal_waypoint_generator_behavioral
```

### 4. Run Full Simulation
Start the complete Gazebo simulation with drone and environment:

```bash
docker compose up gazebo_farm_sim
```

### 5. Process Semantic Data
Generate semantic segmentation and annotations:

```bash
docker compose up semantic_processor
```

## Waypoint Generation Features

### Advanced Algorithms
- **TSP Optimization**: Minimizes total flight distance using nearest-neighbor heuristic
- **Herd Clustering**: DBSCAN-based animal grouping for behavioral strategies  
- **Convex Hull Spiral**: "Onion-layer" approach visiting animals from outside-in
- **Terrain-Aware Safety**: Automatic interpolation of waypoints for collision avoidance

### Terrain Integration
- **LAS Point Cloud Support**: Uses terrain data for collision-free flight paths
- **Texture Mapping**: Proportional texture cropping for realistic visualization
- **Height Safety**: Maintains configurable altitude above terrain features

### Visualization Options
- **Terrain Plots**: Waypoints over LAS point cloud depth maps
- **Texture Plots**: Flight paths over orthophoto backgrounds  
- **Behavioral Plots**: Herd clustering visualization with spiral patterns

## 6. Semantic Segmentation & Bounding Box

The segmentation masks are represented in grayscale, where each pixel value corresponds to a specific class, as defined in the colormap below:

```
COLORMAP = {
    0,  # Unclassified
    1,  # Ground
    2,  # Vegetation
    3,  # Building
    4,  # Water
    5,  # Bridge
    6,  # Bulls & Cows
    7,  # Tractor
    8,  # Pig
    9,  # Horse
    10, # Deer
}
```

In addition, when generating 3D meshes and bounding boxes, each object is also associated with a unique bounding box ID:

```
Bounding Box IDs:
    50, # Pig
    51, # Bulls & Cows
    52, # Horse
    53, # Deer
    54, # Tractor
```