
# Reconstruction ROS Package

## Contents
1. [Dependencies](#1-dependencies)
1. [Launch](#2-launch)
2. [Include](#3-include)
3. [main](#4-main)
4. [YAML](#5-reconstructionyaml)

## 1. dependencies

### Rviz

```
sudo apt-get install ros-noetic-moveit
```

### Realsense

1. Realsense SDK 설치
    
    ```jsx
    
    sudo mkdir -p /etc/apt/keyrings
    
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    
    sudo apt-get install apt-transport-https
    
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    
    sudo tee /etc/apt/sources.list.d/librealsense.list
    
    sudo apt-get update
    
    sudo apt-get install librealsense2-dkms
        
    sudo apt-get install librealsense2-utils
    
    sudo apt-get install librealsense2-dev(optional)
    
    sudo apt-get install librealsense2-dbg(optional)
    ```
    
2. Realsense-ROS 설치
    
    ```jsx
    
    sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
    
    cd ~/catkin_ws/src/
    
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd realsense-ros/
    git checkout git tag | sort -V | grep -P "^2.\\d+\\.\\d+" | tail -1
    cd ..
    
    catkin_init_workspace
    cd ..
    catkin_make clean
    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    catkin_make install
    
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    
    
    

### PCL

```jsx
sudo apt-get update 
sudo apt-get install pcl-tools
```

### CGAL

```jsx
sudo apt-get install libcgal-dev

wget https://github.com/CGAL/cgal/releases/download/v5.6.1/CGAL-5.6.1.tar.xz
tar xf CGAL-5.6.1.tar.xz
cd CGAL-5.6.1

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
```

### VTK

```jsx
sudo apt-get install libvtk7-dev
```

### OPENCV
```
sudo apt install libopencv-dev
```
## 2. Launch
```
<launch>

    <node name="pointcloud_capture" pkg="reconstruction" type="pointcloud_capture"
        output="screen" />

  
    <node name="pointcloud_save" pkg="reconstruction" type="pointcloud_save"
        output="screen" />

    <node name="reconstruction" pkg="reconstruction" type="reconstruction" output="screen" />

 
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find reconstruction)/rviz/rviz_config.rviz"
        output="screen" />

    <node name="keyboard_listener2" pkg="reconstruction" type="keyboard_listener2" output="screen" />


</launch>
  
```
`pointcloud_capture`: Subscribes to point clouds using the Intel Realsense L515 camera. While the realsense2_camera package provided by Realsense can be used, a custom implementation is preferred as it allows control over various parameters such as resolution, IR options, and the application of filters directly within the code.

`pointcloud_save`: This ROS node subscribes to a `keyboard_command` topic, listens for a "capture" command, and saves a single `/camera/depth/points` point cloud as a PCD file with a sequentially numbered filename.

`reconstruction`: This ROS node subscribes to the `keyboard_command` topic and processes point clouds upon receiving the "reconstruction" command. It performs segmentation, outlier removal, downsampling, registration, and meshing to produce a final STL file, while saving intermediate and final PCD files.

`keyboard_listener2`:This ROS node publishes commands (`capture` or `reconstruction`) to the `keyboard_command` topic based on keyboard input (`a` or `b`). It allows triggering specific processes in other nodes via user input.
## 3. Include
1. `Meshing.h`

2. `preprocess.h`
3. `universalRobotsKinematics.h`


## 2.1. Meshing.h
## Overview

The `Meshing` class, implemented in `meshing.h` and `meshing.cpp`, provides functionality for point cloud processing and 3D surface mesh generation using the CGAL library. This module supports operations such as reading and filtering point clouds, normal estimation, bilateral smoothing, and creating surface meshes using the alpha-wrap algorithm. It is designed for robust handling of point cloud data to produce high-quality 3D models.

---

## Dependencies

The `Meshing` module requires the following:

- **CGAL**: Computational Geometry Algorithms Library for advanced geometric processing.
- **C++ STL Libraries**: Includes support for file streams, vectors, and standard exceptions.

---

## Functions

### `Point_set read_points_from_file(const std::string &fname)`

- **Description**: Reads a point cloud from a file and converts it into a `CGAL::Point_set_3`.
- **Parameters**:
  - `fname`: Path to the input file containing point cloud data.
- **Implementation Notes**:
  - Reads the file in binary mode and skips the first 12 lines to handle custom file formats.
  - Extracts 3D points from the remaining lines in the file.
  - Throws a runtime exception if the file cannot be read or if no points are found.
- **Output**: A `Point_set` containing all the points read from the input file.
- **Usage**:
  ```cpp
  Meshing meshing;
  Meshing::Point_set points = meshing.read_points_from_file("input.pcd");
---
### `Point_set outlier_remove(Point_set &points)`

- **Description**: Removes outlier points using a statistical threshold.
- **Parameters**:
  - `points`: Input `Point_set` containing the point cloud.
- **Implementation Notes**:
  - Considers 24 neighbors for each point.
  - Removes the top 2% of points deemed to be outliers.
- **Output**: A filtered `Point_set` with outliers removed.
- **Usage**:
  ```cpp
  points = meshing.outlier_remove(points);
---
### `Point_set grid_simplify(Point_set &points)`

- **Description**: Simplifies the point cloud using a grid-based approach to reduce redundant points.
- **Parameters**:
  - `points`: Input `Point_set` containing the point cloud.
- **Implementation Notes**:
  - Computes the average point spacing using 20 neighbors for each point.
  - Simplifies the point cloud based on 1.2 times the computed spacing.
  - Removes redundant points while maintaining the overall structure.
- **Output**: A simplified `Point_set` with fewer points, retaining the key structure of the input cloud.
- **Usage**:
  ```cpp
  points = meshing.grid_simplify(points);
---
### `PointList convert_to_point_list(Point_set &points)`

- **Description**: Converts a `Point_set` into a `PointList`, which is a vector of point-normal pairs.
- **Parameters**:
  - `points`: Input `Point_set` containing the point cloud.
- **Implementation Notes**:
  - Iterates through all points in the `Point_set`.
  - Initializes normals for each point as zero vectors (`Vector_3(0, 0, 0)`).
  - Stores each point and its corresponding normal in a `PointList`.
- **Output**: A `PointList` containing pairs of points and their initialized normals.
- **Usage**:
  ```cpp
  Meshing::PointList point_list = meshing.convert_to_point_list(points);
---
### `PointList estimate_normal(PointList &points)`

- **Description**: Estimates normals for each point in the point cloud using the jet estimation method.
- **Parameters**:
  - `points`: Input `PointList`, a vector of point-normal pairs.
- **Implementation Notes**:
  - Uses 18 neighbors for each point to estimate normals.
  - Updates the normal vectors in the `PointList` based on the geometric structure of the neighboring points.
  - Leverages CGAL's jet estimation method for robust normal computation.
- **Output**: A `PointList` with estimated normals for each point.
- **Usage**:
  ```cpp
  point_list = meshing.estimate_normal(point_list);
---
### `PointList bilateral_smooth(PointList &points)`

- **Description**: Applies bilateral smoothing to the point cloud to reduce noise while preserving sharp features.
- **Parameters**:
  - `points`: Input `PointList`, a vector of point-normal pairs.
- **Implementation Notes**:
  - Uses a neighborhood size of 200 points for local smoothing.
  - A sharpness angle of 25° is used to control the balance between smoothing and feature preservation.
  - Applies the smoothing operation iteratively for 5 iterations to enhance the result.
  - Updates both the point positions and normals in the `PointList`.
- **Output**: A smoothed `PointList` with reduced noise and preserved sharp features.
- **Usage**:
  ```cpp
  point_list = meshing.bilateral_smooth(point_list);
---
### `void generate_mesh(const PointList &point_list, const std::string &output_filename, double relative_alpha, double relative_offset)`

- **Description**: Generates a 3D surface mesh from the processed point cloud using the alpha-wrap algorithm and saves it as an STL file.
- **Parameters**:
  - `point_list`: Input `PointList` containing points and their normals.
  - `output_filename`: The path to save the generated mesh file (e.g., `.stl` format).
  - `relative_alpha`: Parameter to control the density of the mesh. A larger value results in a denser mesh.
  - `relative_offset`: Parameter to control the smoothness of the mesh. A larger value results in smoother surfaces.
- **Implementation Notes**:
  - Converts the `PointList` into a `Point_container` for compatibility with the alpha-wrap algorithm.
  - Computes the bounding box of the points and calculates the diagonal length.
  - Dynamically determines the alpha and offset values based on the bounding box diagonal length and the provided relative parameters.
  - Uses the alpha-wrap algorithm to generate the mesh.
  - Saves the generated mesh to the specified output file in STL format with high precision.
- **Output**: The surface mesh is saved as an STL file at the specified location.
- **Usage**:
  ```cpp
  meshing.generate_mesh(point_list, "output.stl", 120.0, 1000.0);

---
## 2.2. preprocess.h

## Overview

The `preprocess` module, implemented in `preprocess.h` and `preprocess.cpp`, provides a set of utilities for handling and preprocessing point cloud data using the PCL (Point Cloud Library). It includes functions for filtering, calibration, segmentation, smoothing, downsampling, visualization, and combining multiple point clouds.

---

## Dependencies

- **PCL**: Point Cloud Library for processing and analyzing 3D point cloud data.
- **Eigen**: For matrix transformations.
- **Boost Filesystem**: For handling directories and file operations.

---

## Functions

###  `pcl::PointCloud<pcl::PointNormal> smoothing(const PointCloud::Ptr cloud_src, const float &smoothing_radius)`
- **Description**: Smooths the input point cloud using the Moving Least Squares (MLS) algorithm, preserving features and computing normals.
- **Parameters**:
  - `cloud_src`: Input point cloud.
  - `smoothing_radius`: Radius used for neighbor search during smoothing.
- **Implementation Notes**:
  - Creates a KD-Tree for efficient neighbor search.
  - Computes normals and smooths the surface points.
- **Output**: A smoothed point cloud with normals (`PointNormal`).
- **Usage**:
  ```cpp
  pcl::PointCloud<pcl::PointNormal> smoothed = pre.smoothing(cloud, 0.03);

  ### `PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, const float &downsampleparam)`
---
### `PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, const float &downsampleparam)`

- **Description**: Reduces the density of the point cloud using a voxel grid filter, which aggregates nearby points into a single voxel cell.
- **Parameters**:
  - `cloud_src`: Input point cloud.
  - `downsampleparam`: Size of the voxel grid leaf (i.e., the resolution of the downsampled cloud).
- **Implementation Notes**:
  - The input point cloud is filtered using a voxel grid with the specified leaf size.
  - Reduces the total number of points, preserving the general shape and structure of the cloud.
- **Output**: A downsampled point cloud with fewer points.
- **Usage**:
  ```cpp
  PointCloud::Ptr downsampled = pre.downsampling(cloud, 0.01);
---
### `PointCloud::Ptr statistical_outlier_remove(const PointCloud::Ptr cloud_src, const int &sor_mean, const double &sor_thresh)`

- **Description**: Removes statistical outliers from the point cloud based on mean distance and standard deviation.
- **Parameters**:
  - `cloud_src`: Input point cloud.
  - `sor_mean`: Number of nearest neighbors to consider for each point.
  - `sor_thresh`: Standard deviation multiplier threshold. Points farther than `mean + sor_thresh * stddev` are considered outliers.
- **Implementation Notes**:
  - For each point, computes the average distance to its nearest neighbors.
  - Filters out points that deviate significantly from the local average.
  - Designed to preserve points that conform to the overall point cloud structure.
- **Output**: A point cloud with statistical outliers removed.
- **Usage**:
  ```cpp
  PointCloud::Ptr filtered = pre.statistical_outlier_remove(cloud, 50, 1.0);
---
### `PointCloud::Ptr radius_outlier_remove(const PointCloud::Ptr cloud_src, const double &ror_radius, const int &ror_neighbor)`

- **Description**: Removes outliers based on a radius search, ensuring that each point has a minimum number of neighbors within a given radius.
- **Parameters**:
  - `cloud_src`: Input point cloud.
  - `ror_radius`: Radius within which to search for neighbors.
  - `ror_neighbor`: Minimum number of neighbors required for a point to be considered valid.
- **Implementation Notes**:
  - Points with fewer than `ror_neighbor` neighbors within `ror_radius` are classified as outliers and removed.
  - Maintains the overall density and structure of the point cloud.
- **Output**: A filtered point cloud with radius-based outliers removed.
- **Usage**:
  ```cpp
  PointCloud::Ptr filtered = pre.radius_outlier_remove(cloud, 0.05, 10);
---
### `PointCloud::Ptr calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam)`

- **Description**: Transforms the input point cloud into the robot's world coordinate system using the provided calibration matrices.
- **Parameters**:
  - `cloud_src`: Input point cloud.
  - `base2tcp`: 4x4 transformation matrix from the robot's base to its tool center point (TCP).
  - `tcp2cam`: 4x4 transformation matrix from the TCP to the camera.
- **Implementation Notes**:
  - Sequentially applies the transformations:
    1. Transforms the point cloud from the camera's frame to the TCP frame using `tcp2cam`.
    2. Transforms the point cloud from the TCP frame to the robot base frame using `base2tcp`.
  - Optionally includes a predefined marker-to-base transformation for additional calibration.
- **Output**: A point cloud transformed to the robot's base (world) coordinate system.
- **Usage**:
  ```cpp
  PointCloud::Ptr calibrated_cloud = pre.calibrate(cloud, base2tcp, tcp2cam);
---
### `PointCloud::Ptr calibratemarker2base(const PointCloud::Ptr cloud_src)`

- **Description**: Transforms the input point cloud from the marker's coordinate system to the robot's base coordinate system using a predefined transformation matrix.
- **Parameters**:
  - `cloud_src`: Input point cloud in the marker's coordinate system.
- **Implementation Notes**:
  - Uses a hardcoded 4x4 transformation matrix (`marker2base`) to align the point cloud with the robot base frame.
  - Applies the inverse of the `marker2base` matrix to transform the point cloud to the base frame.
- **Output**: A point cloud transformed to the robot's base coordinate system.
- **Usage**:
  ```cpp
  PointCloud::Ptr calibrated_cloud = pre.calibratemarker2base(cloud);
---
### `PointCloud::Ptr charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[])`

- **Description**: Segments the input point cloud to extract points within a specified bounding box.
- **Parameters**:
  - `cloud_src`: Input point cloud.
  - `min_pt[]`: Array specifying the minimum corner of the bounding box `[x_min, y_min, z_min, w]`.
  - `max_pt[]`: Array specifying the maximum corner of the bounding box `[x_max, y_max, z_max, w]`.
- **Implementation Notes**:
  - Uses a `CropBox` filter to define the bounding box.
  - Points outside the bounding box are removed, leaving only points within the specified range.
- **Output**: A segmented point cloud containing only points within the bounding box.
- **Usage**:
  ```cpp
  float min_pt[] = {0.0, 0.0, 0.0, 1.0};
  float max_pt[] = {1.0, 1.0, 1.0, 1.0};
  PointCloud::Ptr segmented_cloud = pre.charucosegmentation(cloud, min_pt, max_pt);
---
### `void visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src)`

- **Description**: Visualizes multiple point clouds in a single viewer, assigning distinct colors to each cloud for differentiation.
- **Parameters**:
  - `clouds_src`: A vector of input point clouds to be visualized.
- **Implementation Notes**:
  - Creates a PCL Visualizer window with a dark grey background.
  - Assigns unique bright colors to each point cloud for better visualization.
  - Adds coordinate axes to the visualization for reference.
  - Allows interactive inspection of the point clouds until the viewer is closed.
- **Output**: Displays the point clouds in a visualization window.
- **Usage**:
  ```cpp
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = {cloud1, cloud2, cloud3};
  pre.visualizePointClouds(clouds);
---
### `pcl::PointCloud<pcl::PointXYZ>::Ptr PointNormal2PointXYZ(const pcl::PointCloud<pcl::PointNormal> &cloud_src)`

- **Description**: Converts a point cloud with normals (`PointNormal`) to a basic point cloud (`PointXYZ`) by extracting only the position data.
- **Parameters**:
  - `cloud_src`: Input point cloud of type `PointNormal` containing both positions and normals.
- **Implementation Notes**:
  - Iterates through all points in the input cloud.
  - Extracts the `x`, `y`, and `z` coordinates while discarding the normal data.
- **Output**: A new point cloud of type `PointXYZ` containing only the position data.
- **Usage**:
  ```cpp
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud = pre.PointNormal2PointXYZ(normal_cloud);
---
### `std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadData(const std::string &file_paths)`

- **Description**: Loads multiple `.pcd` files from a specified directory and returns them as a vector of point clouds.
- **Parameters**:
  - `file_paths`: Path to the directory containing `.pcd` files.
- **Implementation Notes**:
  - Scans the specified directory for files with the `.pcd` extension.
  - Sorts the files alphabetically to maintain a consistent order.
  - Loads each `.pcd` file into a `PointCloud` and appends it to the output vector.
  - Logs invalid or missing directories as errors.
- **Output**: A vector of `PointCloud::Ptr` objects containing the loaded point clouds.
- **Usage**:
  ```cpp
  std::string folder_path = "/path/to/pcd/folder";
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = pre.loadData(folder_path);
---
### `pcl::PointCloud<pcl::PointXYZ>::Ptr addPoint(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src)`

- **Description**: Merges multiple point clouds into a single point cloud.
- **Parameters**:
  - `clouds_src`: A vector of point clouds (`PointCloud::Ptr`) to be merged.
- **Implementation Notes**:
  - Iteratively adds each point cloud from the input vector to a new result cloud.
  - The resulting cloud contains all the points from the input clouds.
- **Output**: A single point cloud (`PointCloud::Ptr`) that contains all the points from the input clouds.
- **Usage**:
  ```cpp
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud = pre.addPoint(clouds);

## 2.1. universalRobotsKinematics.h
## Overview

The `universalRobotsKinematics` module provides functionality for handling forward kinematics, transformation matrix calculations, YAML configuration loading, and camera-to-marker transformation calculations. It is designed specifically for Universal Robots (UR10/UR10e) and integrates seamlessly with robotic workflows.

---

## Dependencies

- **Eigen**: For matrix operations and transformations.
- **OpenCV**: For Charuco marker detection and pose estimation.
- **Librealsense2**: For camera handling.
- **YAML-CPP**: For parsing YAML configuration files.

---

## Functions

###  `Eigen::Matrix4f forwardKinematics(float theta[])`

- **Description**: Computes the forward kinematics for the UR10/UR10e robotic arm, returning the transformation matrix from the base to the tool center point (TCP).
you can change UR10 or UR10e mode.
- **Parameters**:
  - `theta[]`: Joint angles in degrees for the six joints of the robot.
- **Implementation Notes**:
  - Converts joint angles from degrees to radians.
  - Uses Denavit-Hartenberg parameters specific to UR10/UR10e to compute the transformation matrix.
- **Output**: A 4x4 transformation matrix representing the robot's end-effector pose in the base coordinate system.
- **Usage**:
  ```cpp
  float joint_angles[6] = {0, 45, 30, 90, 0, 45};
  Eigen::Matrix4f tcp_pose = robot.forwardKinematics(joint_angles);
---
### `Eigen::Matrix4f calcTransformationMatrix(float alpha, float a, float d, float theta)`

- **Description**: Computes a single transformation matrix using the Denavit-Hartenberg (DH) parameters.
- **Parameters**:
  - `alpha`: Link twist angle in radians, describing the angle between adjacent z-axes.
  - `a`: Link length, the distance between adjacent z-axes along the x-axis.
  - `d`: Link offset, the distance between adjacent x-axes along the z-axis.
  - `theta`: Joint angle in radians, describing the rotation about the z-axis.
- **Implementation Notes**:
  - Constructs a 4x4 homogeneous transformation matrix using the standard DH parameter equations.
  - Combines rotation and translation transformations into a single matrix.
- **Output**: A 4x4 transformation matrix representing the link's pose relative to the previous link.
- **Usage**:
  ```cpp
  Eigen::Matrix4f T = robot.calcTransformationMatrix(M_PI / 2, 0.5, 0.1, M_PI / 4);
---
### `float deg2rad(float degree)`

- **Description**: Converts an angle from degrees to radians.
- **Parameters**:
  - `degree`: Angle in degrees to be converted.
- **Implementation Notes**:
  - Multiplies the input angle by π/180 to convert it from degrees to radians.
- **Output**: The angle converted to radians.
- **Usage**:
  ```cpp
  float radians = robot.deg2rad(90); // Converts 90 degrees to radians
---
### `void loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ, float &X, float &Y, float &Z, float scene[][6], int &n, float minrange[4], float maxrange[4], float &downsampleparam, int &sor_mean, double &sor_thresh, double &ror_radius, int &ror_neighbor, float &smoothing_radius)`

- **Description**: Loads configuration parameters for robot calibration, scene settings, and preprocessing from a YAML file.
- **Parameters**:
  - `filename`: Path to the YAML configuration file.
  - Reference parameters to be populated with values from the file:
    - `thetaX`, `thetaY`, `thetaZ`: Rotation angles for hand-eye calibration (in radians).
    - `X`, `Y`, `Z`: Translation values for hand-eye calibration.
    - `scene[][6]`: Array to store multiple scene configurations.
    - `n`: Number of scenes provided in the YAML file.
    - `minrange[4]`, `maxrange[4]`: Bounding box limits for segmentation.
    - `downsampleparam`: Voxel grid downsampling parameter.
    - `sor_mean`: Mean K value for statistical outlier removal.
    - `sor_thresh`: Threshold for statistical outlier removal.
    - `ror_radius`: Radius for radius-based outlier removal.
    - `ror_neighbor`: Minimum neighbors for radius-based outlier removal.
    - `smoothing_radius`: Radius for point cloud smoothing.
- **Implementation Notes**:
  - Uses YAML-CPP to parse the YAML file.
  - Converts rotation angles to radians.
  - Iterates through YAML nodes to populate arrays and parameters.
  - Throws exceptions or logs errors if the file format is invalid or parameters are missing.
- **Output**: Populates the provided references with the values from the YAML file.
- **Usage**:
  ```cpp
  float thetaX, thetaY, thetaZ, X, Y, Z;
  float scene[10][6]; // Example size
  int n;
  float minrange[4], maxrange[4], downsampleparam;
  int sor_mean, ror_neighbor;
  double sor_thresh, ror_radius;
  float smoothing_radius;

  robot.loadYAML("config.yaml", thetaX, thetaY, thetaZ, X, Y, Z, scene, n, minrange, maxrange, downsampleparam, sor_mean, sor_thresh, ror_radius, ror_neighbor, smoothing_radius);
---
### `Eigen::Matrix4f createTransformationMatrix(const cv::Mat &rotationMatrix, const cv::Mat &translationVector)`

- **Description**: Constructs a 4x4 transformation matrix by combining a 3x3 rotation matrix and a 3x1 translation vector.
- **Parameters**:
  - `rotationMatrix`: A 3x3 OpenCV matrix representing rotation.
  - `translationVector`: A 3x1 OpenCV matrix representing translation.
- **Implementation Notes**:
  - Converts the OpenCV matrices to Eigen matrices.
  - Places the rotation matrix in the top-left 3x3 block of the transformation matrix.
  - Places the translation vector in the last column of the matrix.
  - Fills the bottom row of the transformation matrix with `[0, 0, 0, 1]`.
- **Output**: A 4x4 Eigen transformation matrix combining rotation and translation.
- **Usage**:
  ```cpp
  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat T = (cv::Mat_<double>(3, 1) << 0.1, 0.2, 0.3);
  Eigen::Matrix4f transformation = robot.createTransformationMatrix(R, T);
---
### `Eigen::Matrix4f calccam2marker(const std::string &filename)`

- **Description**: Computes the transformation matrix from the camera to a Charuco marker board based on a provided image.
- **Parameters**:
  - `filename`: Path to the image file containing the Charuco marker board.
- **Implementation Notes**:
  - Reads the image using OpenCV and detects Aruco markers and Charuco corners in the image.
  - If markers are detected:
    1. Interpolates Charuco corners.
    2. Estimates the pose of the Charuco board using the camera's intrinsic parameters and distortion coefficients.
    3. Converts the pose (rotation and translation) to a 4x4 transformation matrix using the `createTransformationMatrix` function.
  - Returns an identity matrix if the pose estimation fails.
- **Output**: A 4x4 Eigen transformation matrix representing the transformation from the camera to the marker board.
- **Usage**:
  ```cpp
  Eigen::Matrix4f cam_to_marker = robot.calccam2marker("marker_image.jpg");

# 4. Main 

## Overview

This program demonstrates a comprehensive pipeline for processing robotic operations and 3D reconstruction. It integrates the following key functionalities:

1. **Robot Kinematics (universalRobotsKinematics)**:
   - Calculates forward kinematics for the UR10/UR10e robot.
   - Loads calibration parameters from a YAML file.
   - Computes transformation matrices for robotic operations and marker-based calibration.

2. **Point Cloud Preprocessing (preprocess)**:
   - Handles point cloud filtering, downsampling, segmentation, and smoothing.
   - Provides tools for merging multiple point clouds and visualizing results.

3. **3D Meshing (meshing)**:
   - Processes preprocessed point clouds to remove outliers, estimate normals, and smooth surfaces.
   - Generates high-quality 3D meshes using the alpha-wrap algorithm.

By combining these modules, the program allows seamless integration of robot kinematics, point cloud processing, and 3D reconstruction for real-world applications.

## Important parameters
  - `relative_alpha`: determine size of mesh's triangle face. large alpha makes detail meshes.
  - `relative_offset`: determine depth of mesh surface. large offser makes thin mesh surface.
# 5. reconstruction.yaml

## Overview

This YAML file defines the calibration, segmentation, filtering, and preprocessing parameters for a robotic system using a RealSense L515 sensor. The configuration includes settings for hand-eye calibration, bounding box segmentation, point cloud filters, and scene-specific robot poses.

---

## Key Sections

### 1. **`hand_eye_calibration`**
- **Description**: Contains transformation parameters (rotation and translation) for aligning the camera's frame to the robot's coordinate system.
- **Details**:
  - **Format**:
    - First three values: Rotation angles (`thetaX`, `thetaY`, `thetaZ`) in degrees.
    - Last three values: Translation vector (`X`, `Y`, `Z`) in meters.
  - **Example**:
    - **L515 LAB**:
      ```yaml
      - 0.0  # Rotation around X-axis (degrees)
      - 0.0  # Rotation around Y-axis (degrees)
      - 0.0  # Rotation around Z-axis (degrees)
      - 0.00018  # Translation along X-axis (meters)
      - -0.111005  # Translation along Y-axis (meters)
      - 0.04240  # Translation along Z-axis (meters)
      ```

---

### 2. **`minrange` and `maxrange`**
- **Description**: Defines the bounding box limits for point cloud segmentation (basis is ChArUco Marker).
- **Details**:
  - **`minrange`**: Lower bounds of the bounding box in the format `[x_min, y_min, z_min, w]`.
  - **`maxrange`**: Upper bounds of the bounding box in the format `[x_max, y_max, z_max, w]`.
  - **Example**:
    ```yaml
    minrange:
      - 0.4  # Minimum X
      - 0.2  # Minimum Y
      - -0.015  # Minimum Z
      - 1.0  # Homogeneous coordinate
    maxrange:
      - 0.6  # Maximum X
      - 0.4  # Maximum Y
      - 0.2  # Maximum Z
      - 1.0  # Homogeneous coordinate
    ```

---

### 3. **`scene`**
- **Description**: Defines the robot's joint configurations for multiple poses in a scene.
- **Details**:
  - Each scene entry contains six values corresponding to the joint angles of the robot in degrees.
  - **Example**:
    ```yaml
    scene:
      - [36.49, -72.27, -126.33, -69.23, 77.67, -3.76]  # Joint angles for pose 1
      - [40.28, -97.34, -98.32, -107.20, 75.48, 0.02]   # Joint angles for pose 2
    ```

---

### 4. **`downsampleparam`**
- **Description**: Specifies the voxel size for downsampling the point cloud.
- **Details**:
  - Smaller values result in finer downsampling, retaining more detail.
  - **Example**:
    ```yaml
    downsampleparam:
      - 0.003  # Voxel size in meters
    ```

---

### 5. **`sor_mean`**
- **Description**: Defines the number of neighbors considered for Statistical Outlier Removal (SOR).
- **Details**:
  - A higher value increases the robustness of outlier detection but may slow down processing.
  - **Example**:
    ```yaml
    sor_mean:
      - 100  # Number of neighbors
    ```

---

### 6. **`sor_thresh`**
- **Description**: Specifies the standard deviation multiplier for Statistical Outlier Removal.
- **Details**:
  - Points farther than `mean + sor_thresh * stddev` from their neighbors are classified as outliers.
  - **Example**:
    ```yaml
    sor_thresh:
      - 1.5  # Standard deviation multiplier
    ```

---

### 7. **`ror_radius`**
- **Description**: Radius for the Radius Outlier Removal (ROR) filter.
- **Details**:
  - Defines the radius within which neighbors are searched for outlier detection.
  - **Example**:
    ```yaml
    ror_radius:
      - 0.1  # Search radius in meters
    ```

---

### 8. **`ror_neighbor`**
- **Description**: Minimum number of neighbors required within the radius for a point to be classified as an inlier.
- **Details**:
  - Higher values increase the strictness of the filter.
  - **Example**:
    ```yaml
    ror_neighbor:
      - 30  # Minimum neighbors
    ```

---

### 9. **`smoothing_radius`**
- **Description**: Radius for the Moving Least Squares (MLS) smoothing filter.
- **Details**:
  - Smaller values retain finer details in the smoothed point cloud.
  - **Example**:
    ```yaml
    smoothing_radius:
      - 0.01  # Smoothing radius in meters
    ```

---

## Notes
- The YAML file includes configurations for two systems (L515 LAB and L515 AIDIN), with one active configuration uncommented.
- Scene configurations and bounding box ranges should be adjusted based on the specific application.
- Ensure to maintain proper indentation in the YAML file to avoid parsing errors.
