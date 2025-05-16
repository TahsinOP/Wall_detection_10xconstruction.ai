import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def read_point_cloud(file_path):
    """Read point cloud from PCD file."""
    print(f"Reading point cloud from {file_path}...")
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def preprocess_point_cloud(pcd):
    """Preprocess the point cloud by removing outliers and estimating normals."""
    print("Preprocessing point cloud...")
    
    # Remove statistical outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)
    
    # Estimate normals
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=50)
    )
    
    return pcd

def detect_walls(pcd, distance_threshold=0.01, ransac_n=3, num_iterations=1000, min_points=25):
    """Detect walls using RANSAC plane segmentation."""
    print("Detecting walls...")
    
    # Convert to numpy array for easier processing
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    
    # Find vertical planes (walls)
    walls = []
    remaining_points = points.copy()
    remaining_normals = normals.copy()
    
    while len(remaining_points) > ransac_n:
        # Find the best plane
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        if len(inliers) < min_points:  # Minimum number of points to consider as a wall
            break
            
        # Get the plane normal
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        
        # Check if the plane is vertical (normal should be close to horizontal)
        if abs(normal[2]) < 0.5:  # z-component should be small for vertical planes
            walls.append({
                'plane_model': plane_model,
                'inliers': inliers,
                'points': remaining_points[inliers]
            })
            print(f"Found wall with {len(inliers)} points")
        else:
            print(f"Found non-vertical plane with {len(inliers)} points (skipped)")
        
        # Remove the detected plane points
        remaining_points = np.delete(remaining_points, inliers, axis=0)
        remaining_normals = np.delete(remaining_normals, inliers, axis=0)
        
        # Update the point cloud for next iteration
        pcd.points = o3d.utility.Vector3dVector(remaining_points)
        pcd.normals = o3d.utility.Vector3dVector(remaining_normals)
    
    print(f"\nTotal walls detected: {len(walls)}")
    return walls

def visualize_results(pcd, walls):
    """Visualize the point cloud with detected walls highlighted."""
    print("Visualizing results...")
    
    # Create a visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    # Add the original point cloud
    vis.add_geometry(pcd)
    
    # Add detected walls with different colors
    colors = [
        [1, 0, 0],  # Red
        [0, 1, 0],  # Green
        [0, 0, 1],  # Blue
        [1, 1, 0],  # Yellow
        [1, 0, 1],  # Magenta
        [0, 1, 1]   # Cyan
    ]
    
    # Create a list to store all geometries for visualization
    geometries = []
    
    for i, wall in enumerate(walls):
        # Create a point cloud for this wall
        wall_pcd = o3d.geometry.PointCloud()
        wall_pcd.points = o3d.utility.Vector3dVector(wall['points'])
        wall_pcd.paint_uniform_color(colors[i % len(colors)])
        
        # Add to visualization
        vis.add_geometry(wall_pcd)
        geometries.append(wall_pcd)
        
        # Create bounding box for the wall
        bbox = wall_pcd.get_axis_aligned_bounding_box()
        bbox.color = colors[i % len(colors)]
        vis.add_geometry(bbox)
        geometries.append(bbox)
        
        # Add normals visualization
        # Calculate center of the wall points
        center = np.mean(wall['points'], axis=0)
        
        # Get the normal vector from plane equation
        a, b, c, d = wall['plane_model']
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)  # Normalize
        
        # Create normal line
        normal_line = o3d.geometry.LineSet()
        points = o3d.utility.Vector3dVector([center, center + normal * 0.5])  # Scale normal for visibility
        lines = o3d.utility.Vector2iVector([[0, 1]])
        normal_line.points = points
        normal_line.lines = lines
        normal_line.colors = o3d.utility.Vector3dVector([colors[i % len(colors)]])
        
        vis.add_geometry(normal_line)
        geometries.append(normal_line)
        
       
    
    # Set up the view
    vis.get_render_option().point_size = 2
    vis.get_render_option().background_color = [0.1, 0.1, 0.1]
    vis.get_render_option().line_width = 3
    
    # Set up the camera view
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    ctr.set_front([0, 0, -1])
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, -1, 0])
    
    # Capture the image
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image("wall_detection_result.png")
    print("\nSaved visualization as 'wall_detection_result.png'")
    
    # Run the visualization
    vis.run()
    vis.destroy_window()

def main():
    # Read the point cloud
    pcd = read_point_cloud("pointcloud.pcd")
    
    # Preprocess the point cloud
    pcd = preprocess_point_cloud(pcd)
    
    # Detect walls with minimum 50 points threshold
    walls = detect_walls(pcd, min_points=500)
    
    # Print information about detected walls
    print(f"\nDetected {len(walls)} walls:")
    for i, wall in enumerate(walls):
        a, b, c, d = wall['plane_model']
        print(f"Wall {i+1}:")
        print(f"  Plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
        print(f"  Number of points: {len(wall['points'])}")
    
    # Visualize results
    visualize_results(pcd, walls)
    
    # Save the processed point cloud
    o3d.io.write_point_cloud("processed_walls.pcd", pcd)
    print("\nProcessed point cloud saved as 'processed_walls.pcd'")

if __name__ == "__main__":
    main() 