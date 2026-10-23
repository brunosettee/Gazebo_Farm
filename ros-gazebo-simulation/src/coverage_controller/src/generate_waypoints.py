import laspy
import numpy as np
import open3d as o3d
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
from scipy.spatial import KDTree

def load_point_cloud(file_path, scale):
    las = laspy.read(file_path)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    return points * scale

def center_and_normalize_points(points):
    mean_xyz = np.mean(points, axis=0)
    points[:, :3] -= mean_xyz[:3]
    return points

def visualize_point_cloud(points, title="Point Cloud"):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd], window_name=title)

def calculate_grid_size(hfov, vfov, height, overlap):
    # Convert FOV from degrees to radians
    hfov_rad = np.radians(hfov)
    vfov_rad = np.radians(vfov)
    # Calculate the size of the grid based on the FOV and height
    grid_size_x = 2 * height * np.tan(hfov_rad / 2)
    grid_size_y = 2 * height * np.tan(vfov_rad / 2)
    # Adjust grid size based on overlap percentage
    grid_size_x *= (1 - overlap)
    grid_size_y *= (1 - overlap)
    return grid_size_x, grid_size_y

def generate_zigzag_path(points, grid_size_x, grid_size_y, height):
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)
    size_bound = max_bound - min_bound

    print(f"Min-Max {min_bound} {max_bound}")

    # Fix grid_size
    grid_size_x = size_bound[0] / round((size_bound[0] - grid_size_x)/grid_size_x)
    grid_size_y = size_bound[1] / round((size_bound[1] - grid_size_y)/grid_size_y)
    
    # Add offset to avoid points at the borders
    min_bound[0] += grid_size_x / 2
    max_bound[0] -= grid_size_x / 2
    min_bound[1] += grid_size_y / 2
    max_bound[1] -= grid_size_y / 2
    
    x_range = np.arange(min_bound[0], max_bound[0]+0.001, grid_size_x)
    y_range = np.arange(min_bound[1], max_bound[1]+0.001, grid_size_y)
    
    path = []
    toggle = True  # To switch the direction of zigzag

    # Build a KD-Tree for fast nearest-neighbor search
    kdtree = KDTree(points[:, :2])

    for x in tqdm(x_range, desc="Generating zigzag path"):
        y_indices = y_range if toggle else y_range[::-1]
        for y in y_indices:
            # Find points within the current grid cell
            cell_center = np.array([x, y])
            cell_points_idx = kdtree.query_ball_point(cell_center, max(grid_size_x, grid_size_y) / 2)
            cell_points = points[cell_points_idx]
            avg_z = np.mean(cell_points[:, 2]) if len(cell_points) > 0 else 0
            path.append([x, y, avg_z + height])
        toggle = not toggle
    
    return np.array(path)

def subsample_points(points, sample_size=100000):
    if len(points) > sample_size:
        indices = np.random.choice(len(points), sample_size, replace=False)
        points = points[indices]
    return points

def plot_depth_map_with_path(points, path, sample_size=100000, save_path=None):
    points = subsample_points(points, sample_size)
    plt.figure(figsize=(12, 8))
    plt.scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap='viridis', s=1)
    plt.plot(path[:, 0], path[:, 1], 'r-', linewidth=2, label='Zigzag Path')
    plt.scatter(path[:, 0], path[:, 1], c='red', s=15, marker='x')
    
    # Mark start and end
    if len(path) > 0:
        plt.scatter(path[0, 0], path[0, 1], c='green', s=100, marker='s', label='Start')
        plt.scatter(path[-1, 0], path[-1, 1], c='blue', s=100, marker='D', label='End')
    
    plt.colorbar(label='Elevation (z)')
    plt.title('Zigzag Coverage Path over Terrain')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    
    # Save plot if path provided
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    plt.show()

def plot_3d_path(points, path, sample_size=10000):
    points = subsample_points(points, sample_size)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap='viridis', s=1)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', label='Zigzag Path', linewidth=4)
    
    ax.set_title('3D Path Coverage')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Generate an optimized coverage path for a drone from a .las point cloud")
    parser.add_argument("file_path", type=str, help="Path to the .las file")
    #parser.add_argument("--scale", type=float, default=100, help="Scale of the points in .las file")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale of the points in .las file")
    parser.add_argument("--hfov", type=float, default=60.0, help="Horizontal field of view of the camera in degrees")
    parser.add_argument("--vfov", type=float, default=45.0, help="Vertical field of view of the camera in degrees")
    parser.add_argument("--desired_height", type=float, default=10.0, help="Desired height of the camera")
    parser.add_argument("--overlap", type=float, default=0.2, help="Overlap percentage for the grid")
    parser.add_argument("--output_dir", type=str, required=True, help="Output directory for waypoint file")
    parser.add_argument("--plot", action='store_true', help="Show plot of waypoints")
    parser.add_argument("--save_plot", action='store_true', help="Save plot to file")
    args = parser.parse_args()

    points = load_point_cloud(args.file_path, args.scale)
    points = center_and_normalize_points(points)
    # visualize_point_cloud(points, title="Original Point Cloud")

    grid_size_x, grid_size_y = calculate_grid_size(args.hfov, args.vfov, args.desired_height, args.overlap)
    print(f"Calculated grid size: {grid_size_x} x {grid_size_y}")

    path = generate_zigzag_path(points, grid_size_x=grid_size_x, grid_size_y=grid_size_y, height=args.desired_height)

    output_file = args.output_dir.rstrip('/') + '/zigzag_path.txt'
    np.savetxt(output_file, path, fmt="%.3f", delimiter=",", header="x,y,z")
    print(f"Path saved in {output_file}")

    # Plot if requested
    if args.plot or args.save_plot:
        save_path = None
        if args.save_plot:
            save_path = f"{args.output_dir.rstrip('/')}/zigzag_coverage_plot.png"
        
        plot_depth_map_with_path(points, path, save_path=save_path)

if __name__ == "__main__":
    main()
