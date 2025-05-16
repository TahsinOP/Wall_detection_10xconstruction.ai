# Wall Detection in Point Cloud Data

This project implements wall detection in indoor point cloud data using Python and Open3D. The implementation processes a PCD file to identify and visualize vertical planes that represent walls in the environment.

## Requirements

- Python 3.7+
- Open3D
- NumPy
- Matplotlib

Install the required packages using pip:
```bash
pip install open3d numpy matplotlib
```

## Approach

The wall detection process follows these steps:

1. **Point Cloud Reading**: 
   - Reads the input PCD file (accumulated_pcl_site.pcd)
   - Supports any PCD format point cloud

2. **Preprocessing**:
   - Statistical outlier removal:
     - Uses 15 nearest neighbors
     - Removes points that deviate more than 2 standard deviations
   - Normal estimation:
     - Uses 50 nearest neighbors within 0.1 radius
     - Computes surface normals for each point

3. **Wall Detection**:
   - Uses RANSAC (Random Sample Consensus) for plane segmentation
   - Parameters:
     - Distance threshold: 0.01 (maximum distance for a point to be considered part of a plane)
     - RANSAC iterations: 1000
     - Minimum points per wall: 100
   - Identifies vertical planes by checking the normal vector orientation
   - Filters out non-wall planes based on orientation and size

4. **Visualization**:
   - Interactive 3D visualization using Open3D
   - Features:
     - Different colors for each detected wall
     - Bounding boxes around detected walls
     - Normal vectors showing wall orientation
     - Original point cloud shown in the background
   - Saves visualization as 'wall_detection_result.png'

## Usage

Run the script with:
```bash
python wall_detection.py
```

The script will:
1. Process the input point cloud (Input the appropriate .pcd file needed to be processed)
2. Display the detected walls in an interactive 3D viewer
3. Save the processed point cloud as 'processed_walls.pcd'
4. Print detailed information about each wall including:
   - Plane equations
   - Number of points
   - Center points
   - Normal vectors

## Output 

For both the point cloud files pointcloud.pcd and accumulated_pcl_site.pcd

- Interactive 3D visualization showing:
  - Detected walls in different colors
  - Bounding boxes around walls
  - Normal vectors indicating wall orientation
- Processed point cloud file (processed_walls_indoor.pcd and processed_walls_site.pcd)
- Console output with wall plane equations and statistics

## Parameters

The wall detection can be tuned using these parameters in the code:

### Preprocessing Parameters
- `nb_neighbors`: Number of neighbors for outlier removal (default: 15)
- `std_ratio`: Standard deviation ratio for outlier removal (default: 2.0)
- `radius`: Search radius for normal estimation (default: 0.1)
- `max_nn`: Maximum number of neighbors for normal estimation (default: 50)

### Wall Detection Parameters
- `distance_threshold`: Maximum distance for a point to be considered part of a plane (default: 0.01)
- `ransac_n`: Number of points to sample for RANSAC (default: 3)
- `num_iterations`: Number of RANSAC iterations (default: 1000)
- `min_points`: Minimum number of points required to consider a plane as a wall (default: 100) 