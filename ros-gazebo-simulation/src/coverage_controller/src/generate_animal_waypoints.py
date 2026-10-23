import laspy
import numpy as np
import argparse
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
import time
import math
from PIL import Image
from scipy.spatial import ConvexHull

def load_point_cloud(file_path, scale):
    """Load LAS point cloud data"""
    las = laspy.read(file_path)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    return points * scale

def center_and_normalize_points(points):
    """Center and normalize point cloud"""
    mean_xyz = np.mean(points, axis=0)
    points[:, :3] -= mean_xyz[:3]
    return points

def load_texture_image(image_path):
    """Load texture image for terrain reference"""
    try:
        img = Image.open(image_path)
        return np.array(img)
    except:
        return None

def load_animal_coordinates(file_path):
    """Load animal coordinates from coordinates.txt file"""
    animals = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                parts = line.split(', ')
                if len(parts) >= 8:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    # Quaternion components (not used for waypoint generation)
                    qx, qy, qz, qw = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                    animal_type = parts[7].strip()
                    animals.append({
                        'position': [x, y, z],
                        'quaternion': [qx, qy, qz, qw],
                        'type': animal_type
                    })
    return animals

def get_terrain_height(x, y, terrain_points, terrain_kdtree, radius=5.0):
    """Get terrain height at given x,y coordinate using KDTree lookup"""
    try:
        # Find points within radius
        indices = terrain_kdtree.query_ball_point([x, y], radius)
        if len(indices) > 0:
            nearby_points = terrain_points[indices]
            # Return average height of nearby terrain points
            return np.mean(nearby_points[:, 2])
        else:
            # If no points found, expand search radius
            dist, idx = terrain_kdtree.query([x, y], k=1)
            return terrain_points[idx, 2]
    except:
        return 0.0

def generate_naive_animal_waypoints(animals, desired_height, terrain_points=None, terrain_kdtree=None, max_travel_distance=20.0):
    """Naive approach: visit animals in order they appear in file"""
    waypoints = []
    
    print(f"Generating naive waypoints for {len(animals)} animals...")
    
    for i, animal in enumerate(animals):
        x, y, ground_z = animal['position']
        
        # Use terrain data if available, otherwise use animal ground position
        if terrain_points is not None and terrain_kdtree is not None:
            terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
            # Use the higher of terrain height or animal ground position for safety
            safe_ground_height = max(terrain_height, ground_z)
        else:
            safe_ground_height = ground_z
            
        # Set waypoint at desired height above safe ground level
        waypoint_z = safe_ground_height + desired_height
        waypoints.append([x, y, waypoint_z])
    
    waypoints = np.array(waypoints)
    
    # Add safe travel waypoints for terrain collision avoidance
    if terrain_points is not None and terrain_kdtree is not None:
        waypoints, waypoint_types = add_safe_travel_waypoints(waypoints, terrain_points, terrain_kdtree, desired_height, max_travel_distance)
        print(f"Added interpolated waypoints for safe travel. Total waypoints: {len(waypoints)}")
        return waypoints, waypoint_types
    else:
        # All waypoints are animal waypoints when no terrain data
        return waypoints, np.ones(len(waypoints))

def generate_optimized_animal_waypoints(animals, desired_height, terrain_points=None, terrain_kdtree=None, max_travel_distance=20.0, start_pos=None):
    """Optimized approach using Traveling Salesman Problem (TSP) solution"""
    if len(animals) == 0:
        return np.array([]), np.array([])
    
    print(f"Generating optimized waypoints for {len(animals)} animals...")
    
    # Extract positions and add height with terrain awareness
    positions = []
    for animal in animals:
        x, y, ground_z = animal['position']
        
        # Use terrain data if available, otherwise use animal ground position
        if terrain_points is not None and terrain_kdtree is not None:
            terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
            # Use the higher of terrain height or animal ground position for safety
            safe_ground_height = max(terrain_height, ground_z)
        else:
            safe_ground_height = ground_z
            
        waypoint_z = safe_ground_height + desired_height
        positions.append([x, y, waypoint_z])
    
    positions = np.array(positions)
    
    if len(positions) == 1:
        return positions
    
    # If start position not provided, use first animal position
    if start_pos is None:
        start_pos = positions[0]
    
    # Add start position to the mix
    all_positions = np.vstack([start_pos, positions])
    
    # Calculate distance matrix
    dist_matrix = cdist(all_positions, all_positions, metric='euclidean')
    
    # Solve TSP using nearest neighbor heuristic for speed
    waypoints = solve_tsp_nearest_neighbor(all_positions, dist_matrix)
    
    # Remove start position if it was added
    if np.array_equal(waypoints[0], start_pos):
        waypoints = waypoints[1:]
    
    # Add safe travel waypoints for terrain collision avoidance
    if terrain_points is not None and terrain_kdtree is not None:
        waypoints, waypoint_types = add_safe_travel_waypoints(waypoints, terrain_points, terrain_kdtree, desired_height, max_travel_distance)
        print(f"Added interpolated waypoints for safe travel. Total waypoints: {len(waypoints)}")
        return waypoints, waypoint_types
    else:
        # All waypoints are animal waypoints when no terrain data
        return waypoints, np.ones(len(waypoints))

def generate_behavioral_animal_waypoints(animals, desired_height, terrain_points=None, terrain_kdtree=None, cluster_radius=50.0, spiral_resolution=8, max_travel_distance=20.0):
    """Behavioral approach: cluster animals into herds and follow spiral patterns to minimize disturbance"""
    if len(animals) == 0:
        return np.array([]), np.array([])
    
    print(f"Generating behavioral waypoints for {len(animals)} animals...")
    
    # Extract animal positions for clustering
    animal_positions = np.array([animal['position'] for animal in animals])
    
    # Cluster animals into herds using DBSCAN
    print(f"Clustering animals into herds (radius: {cluster_radius}m)...")
    clustering = DBSCAN(eps=cluster_radius, min_samples=2).fit(animal_positions[:, :2])
    
    herds = {}
    for i, label in enumerate(clustering.labels_):
        if label == -1:  # Isolated animals (noise points)
            label = f"isolated_{i}"
        if label not in herds:
            herds[label] = []
        herds[label].append(i)
    
    print(f"Found {len([k for k in herds.keys() if not str(k).startswith('isolated')])} herds and {len([k for k in herds.keys() if str(k).startswith('isolated')])} isolated animals")
    
    # Calculate herd centers for TSP optimization
    herd_data = []
    for herd_id, animal_indices in herds.items():
        herd_animals = [animals[i] for i in animal_indices]
        
        if len(herd_animals) == 1:
            # Single animal
            animal = herd_animals[0]
            center = np.array(animal['position'][:2])
        else:
            # Multiple animals - calculate center
            herd_positions = np.array([animal['position'] for animal in herd_animals])
            center = np.mean(herd_positions[:, :2], axis=0)
        
        herd_data.append({
            'id': herd_id,
            'animals': herd_animals,
            'center': center,
            'size': len(herd_animals)
        })
    
    # Optimize herd visiting order using TSP
    if len(herd_data) > 1:
        print(f"Optimizing herd visiting order using TSP...")
        herd_centers = np.array([herd['center'] for herd in herd_data])
        # Add Z coordinate (use average animal height)
        herd_centers_3d = np.column_stack([
            herd_centers, 
            [np.mean([animal['position'][2] for animal in herd['animals']]) + desired_height for herd in herd_data]
        ])
        
        # Calculate distance matrix for herds
        dist_matrix = cdist(herd_centers_3d, herd_centers_3d, metric='euclidean')
        
        # Solve TSP for herd order
        optimized_centers = solve_tsp_nearest_neighbor(herd_centers_3d, dist_matrix)
        
        # Reorder herd_data based on optimized path
        optimized_herds = []
        for opt_center in optimized_centers:
            for herd in herd_data:
                if np.allclose(herd['center'], opt_center[:2], atol=1e-6):
                    optimized_herds.append(herd)
                    break
        herd_data = optimized_herds
    
    all_waypoints = []
    
    # Process herds in optimized order
    for herd in herd_data:
        herd_animals = herd['animals']
        
        if len(herd_animals) == 1:
            # Single animal - direct approach
            animal = herd_animals[0]
            x, y, ground_z = animal['position']
            
            if terrain_points is not None and terrain_kdtree is not None:
                terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
                safe_ground_height = max(terrain_height, ground_z)
            else:
                safe_ground_height = ground_z
                
            waypoint_z = safe_ground_height + desired_height
            all_waypoints.append([x, y, waypoint_z])
        else:
            # Multiple animals - spiral approach
            spiral_waypoints = generate_spiral_waypoints_for_herd(
                herd_animals, desired_height, terrain_points, terrain_kdtree, spiral_resolution
            )
            all_waypoints.extend(spiral_waypoints)
    
    all_waypoints = np.array(all_waypoints)
    
    # Add safe travel waypoints for terrain collision avoidance
    if terrain_points is not None and terrain_kdtree is not None:
        all_waypoints, waypoint_types = add_safe_travel_waypoints(all_waypoints, terrain_points, terrain_kdtree, desired_height, max_travel_distance)
        print(f"Added interpolated waypoints for safe travel. Total waypoints: {len(all_waypoints)}")
        return all_waypoints, waypoint_types
    else:
        # All waypoints are animal waypoints when no terrain data
        return all_waypoints, np.ones(len(all_waypoints))

def generate_spiral_waypoints_for_herd(herd_animals, desired_height, terrain_points, terrain_kdtree, spiral_resolution):
    """Generate onion-layer spiral waypoints following convex hull layers"""
    remaining_animals = herd_animals.copy()
    all_waypoints = []
    
    while len(remaining_animals) >= 3:  # Need at least 3 points for convex hull
        # Get positions for current layer
        positions = np.array([animal['position'][:2] for animal in remaining_animals])
        
        try:
            # Find convex hull of current layer
            hull = ConvexHull(positions)
            hull_indices = hull.vertices
            
            # Sort hull vertices in counter-clockwise order for smooth path
            hull_points = positions[hull_indices]
            center = np.mean(hull_points, axis=0)
            
            # Sort by angle to create smooth circular path
            angles = np.arctan2(hull_points[:, 1] - center[1], hull_points[:, 0] - center[0])
            sorted_indices = np.argsort(angles)
            
            # Add waypoints for this convex hull layer
            for idx in sorted_indices:
                animal_idx = hull_indices[idx]
                animal = remaining_animals[animal_idx]
                x, y, ground_z = animal['position']
                # Use terrain data if available, otherwise use animal ground position
                if terrain_points is not None and terrain_kdtree is not None:
                    terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
                    # Use the higher of terrain height or animal ground position for safety
                    safe_ground_height = max(terrain_height, ground_z)
                else:
                    safe_ground_height = ground_z
                waypoint_z = safe_ground_height + desired_height
                all_waypoints.append([x, y, waypoint_z])
            
            # Remove hull animals from remaining set for next layer
            remaining_animals = [animal for i, animal in enumerate(remaining_animals) if i not in hull_indices]
            
        except Exception:
            # If convex hull fails, process remaining animals by distance
            if remaining_animals:
                center = np.mean([animal['position'][:2] for animal in remaining_animals], axis=0)
                remaining_animals.sort(key=lambda a: np.linalg.norm(np.array(a['position'][:2]) - center))
                
                for animal in remaining_animals:
                    x, y, ground_z = animal['position']
                    # Use terrain data if available, otherwise use animal ground position
                    if terrain_points is not None and terrain_kdtree is not None:
                        terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
                        # Use the higher of terrain height or animal ground position for safety
                        safe_ground_height = max(terrain_height, ground_z)
                    else:
                        safe_ground_height = ground_z
                    waypoint_z = safe_ground_height + desired_height
                    all_waypoints.append([x, y, waypoint_z])
            break
    
    # Handle remaining animals (less than 3)
    for animal in remaining_animals:
        x, y, ground_z = animal['position']
        # Use terrain data if available, otherwise use animal ground position
        if terrain_points is not None and terrain_kdtree is not None:
            terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
            # Use the higher of terrain height or animal ground position for safety
            safe_ground_height = max(terrain_height, ground_z)
        else:
            safe_ground_height = ground_z
        waypoint_z = safe_ground_height + desired_height
        all_waypoints.append([x, y, waypoint_z])
    
    return all_waypoints

def generate_center_approach_waypoints(center, approach_radius, desired_height, terrain_points, terrain_kdtree, herd_positions):
    """Generate final approach waypoints for detailed herd inspection"""
    approach_points = []
    num_approach_points = 4  # Four cardinal directions
    
    for i in range(num_approach_points):
        angle = i * (2 * math.pi / num_approach_points)
        x = center[0] + approach_radius * math.cos(angle)
        y = center[1] + approach_radius * math.sin(angle)
        
        # Get terrain-aware height
        if terrain_points is not None and terrain_kdtree is not None:
            terrain_height = get_terrain_height(x, y, terrain_points, terrain_kdtree)
            safe_ground_height = max(terrain_height, np.mean(herd_positions[:, 2]))
        else:
            safe_ground_height = np.mean(herd_positions[:, 2])
            
        waypoint_z = safe_ground_height + desired_height
        approach_points.append([x, y, waypoint_z])
    
    return approach_points

def solve_tsp_nearest_neighbor(positions, dist_matrix):
    """Fast nearest neighbor heuristic for TSP"""
    n = len(positions)
    unvisited = set(range(1, n))  # Exclude start position (index 0)
    current = 0
    path = [current]
    
    while unvisited:
        # Find nearest unvisited city
        nearest = min(unvisited, key=lambda x: dist_matrix[current][x])
        path.append(nearest)
        unvisited.remove(nearest)
        current = nearest
    
    # Return positions in optimal order
    return positions[path]

def solve_tsp_2opt(positions, dist_matrix):
    """2-opt improvement for TSP (more sophisticated but slower)"""
    def calculate_route_distance(route, dist_matrix):
        distance = 0
        for i in range(len(route)):
            distance += dist_matrix[route[i]][route[(i + 1) % len(route)]]
        return distance
    
    def two_opt_swap(route, i, k):
        return route[:i] + route[i:k+1][::-1] + route[k+1:]
    
    n = len(positions)
    if n <= 3:
        return solve_tsp_nearest_neighbor(positions, dist_matrix)
    
    # Start with nearest neighbor solution
    route = list(range(n))
    best_distance = calculate_route_distance(route, dist_matrix)
    
    improved = True
    iterations = 0
    max_iterations = min(1000, n * 10)  # Limit iterations for speed
    
    while improved and iterations < max_iterations:
        improved = False
        iterations += 1
        
        for i in range(1, n - 1):
            for k in range(i + 1, n):
                new_route = two_opt_swap(route, i, k)
                new_distance = calculate_route_distance(new_route, dist_matrix)
                
                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance
                    improved = True
                    break
            if improved:
                break
    
    return positions[route]

def calculate_total_distance(waypoints):
    """Calculate total distance traveled"""
    if len(waypoints) < 2:
        return 0
    
    total_dist = 0
    for i in range(len(waypoints) - 1):
        dist = np.linalg.norm(waypoints[i+1] - waypoints[i])
        total_dist += dist
    
    return total_dist

def interpolate_safe_waypoints(start_point, end_point, terrain_points, terrain_kdtree, desired_height, max_distance=20.0, step_size=10.0):
    """Create interpolated waypoints between two points to avoid terrain collisions"""
    start = np.array(start_point[:2])  # x, y only
    end = np.array(end_point[:2])      # x, y only
    
    # Calculate distance between points
    distance = np.linalg.norm(end - start)
    
    # If distance is small, no interpolation needed
    if distance <= max_distance:
        return []
    
    # Calculate number of intermediate steps
    num_steps = int(np.ceil(distance / step_size))
    interpolated_waypoints = []
    
    for i in range(1, num_steps):  # Skip start (0) and end (num_steps)
        # Linear interpolation between start and end
        alpha = i / num_steps
        interp_x = start[0] + alpha * (end[0] - start[0])
        interp_y = start[1] + alpha * (end[1] - start[1])
        
        # Get terrain height at interpolated point
        if terrain_points is not None and terrain_kdtree is not None:
            terrain_height = get_terrain_height(interp_x, interp_y, terrain_points, terrain_kdtree)
        else:
            # Use linear interpolation of Z coordinates if no terrain data
            terrain_height = start_point[2] + alpha * (end_point[2] - start_point[2])
        
        # Set waypoint at desired height above terrain
        interp_z = terrain_height + desired_height
        interpolated_waypoints.append([interp_x, interp_y, interp_z])
    
    return interpolated_waypoints

def add_safe_travel_waypoints(waypoints, terrain_points, terrain_kdtree, desired_height, max_distance=20.0):
    """Add interpolated waypoints between distant animal waypoints for safe travel"""
    if len(waypoints) < 2:
        return waypoints, np.ones(len(waypoints))  # All are animal waypoints
    
    safe_waypoints = [waypoints[0]]  # Start with first waypoint
    waypoint_types = [1]  # First waypoint is animal type
    
    for i in range(1, len(waypoints)):
        current_point = waypoints[i-1]
        next_point = waypoints[i]
        
        # Add interpolated waypoints if needed
        interpolated = interpolate_safe_waypoints(
            current_point, next_point, terrain_points, terrain_kdtree, 
            desired_height, max_distance
        )
        
        # Add interpolated waypoints (type 0 = intermediate)
        safe_waypoints.extend(interpolated)
        waypoint_types.extend([0] * len(interpolated))
        
        # Add the target waypoint (type 1 = animal)
        safe_waypoints.append(next_point)
        waypoint_types.append(1)
    
    return np.array(safe_waypoints), np.array(waypoint_types)

def save_waypoints(waypoints, output_file, waypoint_types=None):
    """Save waypoints to file with optional type metadata"""
    if waypoint_types is not None:
        # Save with type information: x,y,z,type (0=intermediate, 1=animal)
        data = np.column_stack([waypoints, waypoint_types])
        np.savetxt(output_file, data, fmt="%.3f", delimiter=",", header="x,y,z,type")
    else:
        # Original format for backward compatibility
        np.savetxt(output_file, waypoints, fmt="%.3f", delimiter=",", header="x,y,z")
    print(f"Animal waypoints saved to {output_file}")

def subsample_points(points, sample_size=100000):
    """Subsample points for efficient plotting"""
    if len(points) > sample_size:
        indices = np.random.choice(len(points), sample_size, replace=False)
        points = points[indices]
    return points

def plot_depth_map_with_path(terrain_points, waypoints, title="Animal Waypoints over Terrain", save_path=None, sample_size=100000):
    """Plot waypoints over terrain depth map (like zigzag version)"""
    if terrain_points is not None:
        # Crop terrain points to waypoint bounds with margin
        waypoints_array = np.array(waypoints)
        margin = 50.0  # 50m margin
        
        x_min, x_max = waypoints_array[:, 0].min() - margin, waypoints_array[:, 0].max() + margin
        y_min, y_max = waypoints_array[:, 1].min() - margin, waypoints_array[:, 1].max() + margin
        
        # Filter terrain points within bounds
        mask = ((terrain_points[:, 0] >= x_min) & (terrain_points[:, 0] <= x_max) & 
                (terrain_points[:, 1] >= y_min) & (terrain_points[:, 1] <= y_max))
        terrain_points = terrain_points[mask]
        
        terrain_points = subsample_points(terrain_points, sample_size)
        
    plt.figure(figsize=(12, 8))
    
    # Plot terrain if available (invert X and Y axes)
    if terrain_points is not None:
        plt.scatter(-terrain_points[:, 0], -terrain_points[:, 1], c=terrain_points[:, 2], 
                   cmap='viridis', s=1, alpha=0.6)
        plt.colorbar(label='Elevation (z)')
    
    # Plot waypoints (invert X and Y axes)
    plt.plot(-waypoints[:, 0], -waypoints[:, 1], 'r-', linewidth=2, label='Flight Path')
    plt.scatter(-waypoints[:, 0], -waypoints[:, 1], c='red', s=15, marker='x')
    
    # Mark start and end (invert X and Y axes)
    if len(waypoints) > 0:
        plt.scatter(-waypoints[0, 0], -waypoints[0, 1], c='green', s=100, marker='s', label='Start')
        plt.scatter(-waypoints[-1, 0], -waypoints[-1, 1], c='blue', s=100, marker='D', label='End')
    
    plt.title(title)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    plt.gca().invert_xaxis()  # Flip figure horizontally
    
    # Save plot if path provided
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    plt.show()

def plot_texture_with_path(texture_image, terrain_bounds, waypoints, title="Animal Waypoints over Texture", save_path=None):
    """Plot waypoints over texture image with proper proportional cropping"""
    waypoints_array = np.array(waypoints)
    margin = 50.0  # 50m margin
    
    # Calculate waypoint bounds
    x_min, x_max = waypoints_array[:, 0].min() - margin, waypoints_array[:, 0].max() + margin
    y_min, y_max = waypoints_array[:, 1].min() - margin, waypoints_array[:, 1].max() + margin
    
    # Calculate terrain bounds from point cloud
    terrain_x_min, terrain_x_max = terrain_bounds['x_min'], terrain_bounds['x_max']
    terrain_y_min, terrain_y_max = terrain_bounds['y_min'], terrain_bounds['y_max']
    
    # Calculate proportional texture crop
    texture_height, texture_width = texture_image.shape[:2]
    
    # Map waypoint bounds to texture pixel coordinates
    x_ratio = (x_min - terrain_x_min) / (terrain_x_max - terrain_x_min)
    x_ratio_max = (x_max - terrain_x_min) / (terrain_x_max - terrain_x_min)
    y_ratio = (y_min - terrain_y_min) / (terrain_y_max - terrain_y_min)
    y_ratio_max = (y_max - terrain_y_min) / (terrain_y_max - terrain_y_min)
    
    # Crop texture proportionally
    crop_x_min = max(0, int(x_ratio * texture_width))
    crop_x_max = min(texture_width, int(x_ratio_max * texture_width))
    crop_y_min = max(0, int((1 - y_ratio_max) * texture_height))  # Flip Y for image coordinates
    crop_y_max = min(texture_height, int((1 - y_ratio) * texture_height))
    
    cropped_texture = texture_image[crop_y_min:crop_y_max, crop_x_min:crop_x_max]
    
    # Calculate figure size proportional to cropped area
    aspect_ratio = (x_max - x_min) / (y_max - y_min)
    if aspect_ratio > 1:
        fig_width = 12
        fig_height = 12 / aspect_ratio
    else:
        fig_width = 12 * aspect_ratio
        fig_height = 12
    
    plt.figure(figsize=(fig_width, fig_height))
    
    # Display cropped texture (invert coordinates)
    plt.imshow(cropped_texture, extent=[-x_max, -x_min, -y_max, -y_min], origin='lower', alpha=0.8)
    
    # Plot waypoints (invert X and Y axes)
    if len(waypoints) > 0:
        plt.plot(-waypoints_array[:, 0], -waypoints_array[:, 1], 'r-', linewidth=2, alpha=0.9, label='Flight Path')
        plt.scatter(-waypoints_array[:, 0], -waypoints_array[:, 1], c='red', s=50, zorder=5, label='Waypoints')
        
        # Mark start and end points (invert X and Y axes)
        plt.scatter(-waypoints_array[0, 0], -waypoints_array[0, 1], c='green', s=100, marker='s', zorder=6, label='Start')
        plt.scatter(-waypoints_array[-1, 0], -waypoints_array[-1, 1], c='orange', s=100, marker='s', zorder=6, label='End')
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.gca().invert_xaxis()  # Flip figure horizontally
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    plt.show()

def plot_animal_waypoints(animals, waypoints, title="Animal Waypoints", save_path=None):
    """Plot waypoints and animal positions"""
    plt.figure(figsize=(12, 8))
    
    # Plot animal positions (invert X and Y axes)
    animal_positions = np.array([animal['position'] for animal in animals])
    animal_types = [animal['type'] for animal in animals]
    
    # Color code by animal type
    type_colors = {'pig.dae': 'pink', 'bull_1.dae': 'brown', 'deer.dae': 'orange', 
                   'cow_1.dae': 'black', 'cow_2.dae': 'gray'}
    
    for atype, color in type_colors.items():
        mask = np.array([t == atype for t in animal_types])
        if np.any(mask):
            pos = animal_positions[mask]
            plt.scatter(-pos[:, 0], -pos[:, 1], c=color, label=atype.replace('.dae', ''), 
                       s=50, alpha=0.7, marker='o')
    
    # Plot waypoints (invert X and Y axes)
    plt.plot(-waypoints[:, 0], -waypoints[:, 1], 'r-', linewidth=2, label='Flight Path')
    plt.scatter(-waypoints[:, 0], -waypoints[:, 1], c='red', s=30, marker='x', label='Waypoints')
    
    # Mark start and end (invert X and Y axes)
    if len(waypoints) > 0:
        plt.scatter(-waypoints[0, 0], -waypoints[0, 1], c='green', s=100, marker='s', label='Start')
        plt.scatter(-waypoints[-1, 0], -waypoints[-1, 1], c='blue', s=100, marker='D', label='End')
    
    plt.title(title)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.gca().invert_xaxis()  # Flip figure horizontally
    
    # Save plot if path provided
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    plt.show()

def plot_behavioral_waypoints(animals, waypoints, title="Behavioral Animal Waypoints", save_path=None, cluster_radius=50.0):
    """Plot waypoints with herd clustering visualization"""
    plt.figure(figsize=(14, 10))
    
    # Extract animal positions for clustering visualization
    animal_positions = np.array([animal['position'] for animal in animals])
    animal_types = [animal['type'] for animal in animals]
    
    # Perform clustering to show herds
    clustering = DBSCAN(eps=cluster_radius, min_samples=2).fit(animal_positions[:, :2])
    
    # Color code by herd
    unique_labels = set(clustering.labels_)
    colors = plt.cm.Set3(np.linspace(0, 1, len(unique_labels)))
    
    for k, color in zip(unique_labels, colors):
        if k == -1:
            # Isolated animals in black
            color = 'black'
            marker = 'x'
            label = 'Isolated Animals'
        else:
            marker = 'o'
            label = f'Herd {k}'
        
        class_member_mask = (clustering.labels_ == k)
        xy = animal_positions[class_member_mask]
        
        if len(xy) > 0:
            # Invert X and Y axes for animal positions
            plt.scatter(-xy[:, 0], -xy[:, 1], c=[color], marker=marker, s=60, 
                       alpha=0.8, label=label, edgecolors='black', linewidth=0.5)
            
            # Draw herd boundary for clusters (invert coordinates)
            if k != -1 and len(xy) > 1:
                center = np.mean(xy, axis=0)
                distances = np.linalg.norm(xy[:, :2] - center[:2], axis=1)
                radius = np.max(distances) + 20  # Same buffer as in algorithm
                
                circle = plt.Circle((-center[0], -center[1]), radius, 
                                  color=color, fill=False, linestyle='--', 
                                  linewidth=2, alpha=0.6, label=f'Herd {k} boundary')
                plt.gca().add_patch(circle)
    
    # Plot waypoints with spiral pattern indication (invert X and Y axes)
    if len(waypoints) > 0:
        plt.plot(-waypoints[:, 0], -waypoints[:, 1], 'r-', linewidth=1.5, alpha=0.7, label='Flight Path')
        plt.scatter(-waypoints[:, 0], -waypoints[:, 1], c='red', s=20, marker='+', label='Waypoints')
        
        # Mark start and end (invert X and Y axes)
        plt.scatter(-waypoints[0, 0], -waypoints[0, 1], c='green', s=120, marker='s', 
                   label='Start', edgecolors='black', linewidth=1)
        plt.scatter(-waypoints[-1, 0], -waypoints[-1, 1], c='blue', s=120, marker='D', 
                   label='End', edgecolors='black', linewidth=1)
    
    plt.title(title)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.gca().invert_xaxis()  # Flip figure horizontally
    
    # Save plot if path provided
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Generate animal-focused waypoints for drone coverage")
    parser.add_argument("coordinates_file", type=str, help="Path to coordinates.txt file")
    parser.add_argument("--las_file", type=str, help="Path to LAS file for terrain data")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor for LAS coordinates")
    parser.add_argument("--strategy", type=str, choices=['naive', 'optimized', 'behavioral'], default='optimized',
                        help="Waypoint generation strategy")
    parser.add_argument("--desired_height", type=float, default=10.0, 
                        help="Desired height above ground for waypoints")
    parser.add_argument("--output_dir", type=str, required=True,
                        help="Output directory for waypoint file")
    parser.add_argument("--plot", action='store_true', help="Show plot of waypoints")
    parser.add_argument("--save_plot", action='store_true', help="Save plot to file")
    parser.add_argument("--texture_file", type=str, help="Path to texture image file (textura.jpeg)")
    parser.add_argument("--plot_texture", action='store_true', help="Plot texture background instead of animal types")
    parser.add_argument("--plot_terrain", action='store_true', help="Plot terrain depth map instead of simple waypoint plot")
    parser.add_argument("--cluster_radius", type=float, default=50.0, help="Radius for animal clustering in behavioral strategy (meters)")
    parser.add_argument("--spiral_resolution", type=int, default=8, help="Number of waypoints per spiral turn in behavioral strategy")
    parser.add_argument("--max_travel_distance", type=float, default=20.0, help="Maximum distance between waypoints before adding intermediate points for terrain safety (meters)")
    args = parser.parse_args()

    # Load animal coordinates
    animals = load_animal_coordinates(args.coordinates_file)
    print(f"Loaded {len(animals)} animal positions")
    
    if len(animals) == 0:
        print("No animals found in coordinates file!")
        return
    
    # Load terrain data if provided
    terrain_points = None
    terrain_kdtree = None
    if args.las_file:
        print(f"Loading terrain data from {args.las_file}...")
        terrain_points = load_point_cloud(args.las_file, args.scale)
        terrain_points = center_and_normalize_points(terrain_points)
        # Build KD-Tree for fast terrain height lookup
        terrain_kdtree = KDTree(terrain_points[:, :2])
        print(f"Loaded {len(terrain_points)} terrain points")
    else:
        print("No terrain file provided - using animal ground positions only")
    
    # Count animals by type
    animal_types = {}
    for animal in animals:
        atype = animal['type']
        animal_types[atype] = animal_types.get(atype, 0) + 1
    
    print("Animal distribution:")
    for atype, count in animal_types.items():
        print(f"  {atype.replace('.dae', '')}: {count}")
    
    # Generate waypoints based on strategy
    start_time = time.time()
    
    if args.strategy == 'naive':
        waypoints, waypoint_types = generate_naive_animal_waypoints(animals, args.desired_height, terrain_points, terrain_kdtree, args.max_travel_distance)
    elif args.strategy == 'behavioral':
        waypoints, waypoint_types = generate_behavioral_animal_waypoints(animals, args.desired_height, terrain_points, terrain_kdtree, 
                                                       args.cluster_radius, args.spiral_resolution, args.max_travel_distance)
    else:  # optimized
        waypoints, waypoint_types = generate_optimized_animal_waypoints(animals, args.desired_height, terrain_points, terrain_kdtree, args.max_travel_distance)
    
    generation_time = time.time() - start_time
    
    # Calculate metrics
    total_distance = calculate_total_distance(waypoints)
    
    print(f"\nWaypoint generation completed in {generation_time:.3f} seconds")
    print(f"Strategy: {args.strategy}")
    print(f"Number of waypoints: {len(waypoints)}")
    print(f"Total flight distance: {total_distance:.2f} meters")
    print(f"Average distance per waypoint: {total_distance/max(1, len(waypoints)-1):.2f} meters")
    
    # Save waypoints with type information
    output_file = f"{args.output_dir.rstrip('/')}/animal_waypoints_{args.strategy}.txt"
    save_waypoints(waypoints, output_file, waypoint_types)
    
    # Load texture if provided
    texture_image = None
    terrain_bounds = None
    if args.texture_file:
        print(f"Loading texture from {args.texture_file}...")
        texture_image = load_texture_image(args.texture_file)
        if texture_image is not None and terrain_points is not None:
            # Calculate terrain bounds for texture mapping
            terrain_bounds = {
                'x_min': terrain_points[:, 0].min(),
                'x_max': terrain_points[:, 0].max(),
                'y_min': terrain_points[:, 1].min(),
                'y_max': terrain_points[:, 1].max()
            }
            print(f"Loaded texture image: {texture_image.shape}")
    
    # Plot if requested
    if args.plot or args.save_plot:
        save_path = None
        if args.save_plot:
            if args.plot_texture and texture_image is not None:
                save_path = f"{args.output_dir.rstrip('/')}/animal_waypoints_{args.strategy}_texture_plot.png"
            elif args.plot_terrain and terrain_points is not None:
                save_path = f"{args.output_dir.rstrip('/')}/animal_waypoints_{args.strategy}_terrain_plot.png"
            else:
                save_path = f"{args.output_dir.rstrip('/')}/animal_waypoints_{args.strategy}_plot.png"
        
        # Choose plotting method based on arguments
        if args.plot_texture and texture_image is not None and terrain_bounds is not None:
            # Plot with texture background
            plot_texture_with_path(texture_image, terrain_bounds, waypoints, 
                                 f"Animal Waypoints over Texture - {args.strategy.capitalize()} Strategy",
                                 save_path=save_path)
        elif args.plot_terrain and terrain_points is not None:
            # Plot terrain depth map style
            plot_depth_map_with_path(terrain_points, waypoints, 
                                   f"Animal Waypoints over Terrain - {args.strategy.capitalize()} Strategy",
                                   save_path=save_path)
        elif args.strategy == 'behavioral':
            # Plot behavioral clustering and spiral patterns
            plot_behavioral_waypoints(animals, waypoints, 
                                    f"Behavioral Animal Waypoints - Herd-Aware Spiral Strategy",
                                    save_path=save_path, cluster_radius=args.cluster_radius)
        else:
            # Plot animal type style
            plot_animal_waypoints(animals, waypoints, 
                                f"Animal Waypoints - {args.strategy.capitalize()} Strategy",
                                save_path=save_path)

if __name__ == "__main__":
    main()
