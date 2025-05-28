# Livox Mapping (ROS 2)

Livox_mapping is a ROS 2 package for 3D SLAM using Livox LiDARs, based on the LOAM algorithm. It provides real-time odometry and mapping capabilities. This version has been adapted for ROS 2.

<div align="center">
    <img src="doc/results/mid40_hall.gif" width = 45% >
    <img src="doc/results/horizon_parking.gif" width = 45% >
</div>

## Features (and Original Key Issues/Goals)
This package aims to provide robust LOAM-based SLAM for Livox Lidars. Original goals from the ROS 1 version included:
1. Support for multiple Livox LiDAR models.
2. Specialized feature extraction for Livox LiDAR patterns.
3. Odometry removal for small FOV situations (this may refer to specific algorithm tweaks within LOAM).

## 1. Prerequisites

### 1.1. System Requirements
*   Ubuntu 22.04 (or compatible)
*   ROS 2 (e.g., Humble, Jazzy - please adapt to your ROS 2 version)
*   C++17 compiler (as specified in `CMakeLists.txt`)

### 1.2. External Libraries (System Dependencies)
*   **PCL (Point Cloud Library):** Version 1.10 or higher. Installation instructions: [PCL Downloads](http://www.pointclouds.org/downloads/linux.html).
    ```bash
    sudo apt-get update
    sudo apt-get install libpcl-dev
    ```
*   **Eigen3:** Version 3.3 or higher. Usually installed with PCL or ROS. If not:
    ```bash
    sudo apt-get install libeigen3-dev
    ```
*   **OpenCV:** Version 4.x. Usually installed with ROS Desktop Full. If not:
    ```bash
    sudo apt-get install libopencv-dev python3-opencv
    ```
*   **OpenMP:** (Optional, for parallelization) - Usually available with GCC.

### 1.3. ROS 2 Dependencies
*   **livox_ros_driver2:** Ensure you have installed `livox_ros_driver2` for your specific Livox LiDAR model and ROS 2 version. Follow instructions from [livox_ros_driver2 GitHub](https://github.com/Livox-SDK/livox_ros_driver2).
*   Other ROS 2 packages (will be installed via `rosdep`): `rclcpp`, `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `pcl_conversions`, `ament_cmake`.

## 2. Build Instructions

1.  **Create/Navigate to your ROS 2 Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone the Repository:**
    ```bash
    git clone https://github.com/Livox-SDK/livox_mapping.git 
    # Or your specific fork/branch
    ```

3.  **Install Dependencies:**
    ```bash
    cd ~/ros2_ws
    sudo apt-get update 
    rosdep install --from-paths src --ignore-src -r -y
    ```
    This command installs ROS dependencies listed in `package.xml`. Ensure `rosdep` is initialized (`sudo rosdep init` and `rosdep update`) if you haven't used it before.

4.  **Build the Package:**
    ```bash
    colcon build --symlink-install
    ```

5.  **Source the Workspace:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    # Or add this to your .bashrc for convenience
    ```

*Remarks:*
- If you want to save the PCD map file, ensure the `map_file_path` parameter in the launch file or C++ node points to a valid directory with write permissions. The default in `mapping_mid.launch.py` is " " (an empty space string), which likely means the `laserMapping` node will use an internal default or not save unless specified.

## 3. Package Architecture

The `livox_mapping` package consists of several key nodes that work together in a LOAM pipeline:

*   **`livox_repub` (Executable: `livox_repub`)**
    *   **Purpose:** Subscribes to raw Livox custom messages (`livox_ros_driver2::msg::CustomMsg`), aggregates them, and republishes as a standard ROS 2 point cloud (`sensor_msgs::msg::PointCloud2`).
    *   **Point Type:** Converts Livox points to `pcl::PointXYZINormal`. Critically, it populates:
        *   `intensity`: Encodes Livox line number (integer part) and reflectivity (decimal part).
        *   `curvature`: Encodes a normalized timestamp for points within the aggregated scan (scaled by 0.1). This is vital for motion distortion compensation.
*   **`scanRegistration` (Executable: `loam_scanRegistration`)**
    *   **Purpose:** Processes point clouds from `livox_repub` (or directly from a driver if configured) to extract geometric features: sharp edges (corners) and planar surfaces.
    *   **Output:** Publishes feature clouds (`/laser_cloud_sharp`, `/laser_cloud_flat`).
    *   **Point Type:** Uses `pcl::PointXYZINormal`. It preserves `intensity` (line number + reflectivity) and `curvature` (normalized timestamp) data from the input cloud (e.g., from `livox_repub`), making it suitable for use in a pipeline with `laserMapping` for accurate motion compensation.
*   **`scanRegistration_horizon` (Executable: `loam_scanRegistration_horizon`)**
    *   **Purpose:** An alternative feature extraction node, potentially optimized for Livox Horizon or multi-line Lidars.
    *   **Point Type:** Correctly uses `pcl::PointXYZINormal` and preserves `intensity` and `curvature` from the input cloud (e.g., from `livox_repub`). This is also a recommended feature extractor to use with `laserMapping`.
*   **`laserMapping` (Executable: `loam_laserMapping`)**
    *   **Purpose:** Performs scan-to-map matching using the extracted features. It estimates the LiDAR's pose, corrects for motion distortion, and builds/updates a 3D map.
    *   **Point Type:** Expects input feature clouds with `pcl::PointXYZINormal` points, specifically utilizing the `curvature` field for motion distortion compensation (assuming it's `normalized_scan_time * 0.1` from upstream nodes).

**Recommended Data Flow for Accurate Mapping:**
`livox_ros_driver2` -> `livox_repub` (outputs `/livox_pcl0` with `PointXYZINormal`) -> `loam_scanRegistration` or `loam_scanRegistration_horizon` (consumes `/livox_pcl0`, outputs feature clouds with `PointXYZINormal`) -> `loam_laserMapping`.

## 4. Parameters

Parameters can be set via launch files or command-line arguments.

### `livox_repub` Node
*   `to_merge_count` (int, default: `1`): Number of input `livox_ros_driver2::msg::CustomMsg` messages to accumulate before publishing an aggregated point cloud.

### `scanRegistration_horizon` Node
*   `n_scans` (int, default: `6`): Number of scan lines/rings assumed for feature extraction. This should be set according to your LiDAR characteristics or how `livox_repub` encodes line information into the intensity field.

### `laserMapping` Node (from `mapping_mid.launch.py`)
*   `map_file_path` (string, default: `" "`): Path to save the generated PCD map files. If left as an empty space or not specified, saving might be disabled or use an internal default.
*   `filter_parameter_corner` (double, default: `0.1` in `mapping_mid.launch.py`, `0.2` in node source): Voxel grid leaf size for downsampling corner point clouds.
*   `filter_parameter_surf` (double, default: `0.2` in `mapping_mid.launch.py`, `0.4` in node source): Voxel grid leaf size for downsampling surface point clouds.

*(Note: Default values might differ between launch files and C++ source code. Launch file values take precedence if set.)*

## 5. Topics & Messages

### `livox_repub` Node
*   **Subscribes:**
    *   `/livox/lidar` (`livox_ros_driver2::msg::CustomMsg`): Raw Livox point data.
*   **Publishes:**
    *   `/livox_pcl0` (`sensor_msgs::msg::PointCloud2`): Processed cloud with `pcl::PointXYZINormal` points, including intensity (line# + reflectivity) and curvature (timestamp_norm * 0.1).

### `scanRegistration` / `scanRegistration_horizon` Node
*   **Subscribes:**
    *   `/livox_pcl0` (`sensor_msgs::msg::PointCloud2`): Input point cloud, expected to contain `pcl::PointXYZINormal` from `livox_repub`.
*   **Publishes:**
    *   `/livox_cloud` (or `/livox_cloud_horizon`) (`sensor_msgs::msg::PointCloud2`): The input cloud after some initial processing/filtering by this node.
    *   `/laser_cloud_sharp` (or `/laser_cloud_sharp_horizon`) (`sensor_msgs::msg::PointCloud2`): Extracted corner feature points.
    *   `/laser_cloud_flat` (or `/laser_cloud_flat_horizon`) (`sensor_msgs::msg::PointCloud2`): Extracted surface feature points.

### `laserMapping` Node
*   **Subscribes:**
    *   `/laser_cloud_sharp` (or `/laser_cloud_sharp_horizon`) (`sensor_msgs::msg::PointCloud2`): Corner points.
    *   `/laser_cloud_flat` (or `/laser_cloud_flat_horizon`) (`sensor_msgs::msg::PointCloud2`): Surface points.
    *   `/livox_cloud` (or `/livox_cloud_horizon`) (`sensor_msgs::msg::PointCloud2`): Full resolution input cloud for registration and map building.
*   **Publishes:**
    *   `/laser_cloud_surround` (`sensor_msgs::msg::PointCloud2`): Point cloud of the local map surrounding the current pose.
    *   `/laser_cloud_surround_corner` (`sensor_msgs::msg::PointCloud2`): Corner points from the local map. (Note: Original README did not list this, but it's often present in LOAM systems).
    *   `/velodyne_cloud_registered` (`sensor_msgs::msg::PointCloud2`): Full resolution cloud registered to the map (effectively the deskewed and map-aligned current scan). Topic name might be a remnant from original LOAM.
    *   `/aft_mapped_to_init` (`nav_msgs::msg::Odometry`): Estimated odometry of the LiDAR in the map frame (`camera_init`).

## 6. Services
No ROS 2 services are explicitly defined by these nodes in the reviewed C++ code.

## 7. Usage (Running the Demo)

### 7.1. Launching the System
Ensure `livox_ros_driver2` is running and publishing Livox data. The specific launch command for `livox_ros_driver2` depends on your LiDAR model (e.g., Mid-40, Horizon, Avia) and its configuration. Refer to the `livox_ros_driver2` documentation.

Example for `mapping_mid.launch.py` (typically for Mid-series Lidars):
```bash
# Terminal 1: Source workspace and launch your Livox driver
# e.g., ros2 launch livox_ros_driver2 msg_MID360_launch.py
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 <your_lidar_specific_launch_file.launch.py>

# Terminal 2: Source workspace and launch livox_mapping
source ~/ros2_ws/install/setup.bash
ros2 launch livox_mapping mapping_mid.launch.py rviz:=true 
```

**Important Notes on Launching:**
*   The `mapping_mid.launch.py` file currently launches `loam_scanRegistration` and `loam_laserMapping`. This setup should now work correctly due to fixes in `loam_scanRegistration`.
*   You may need to create or adapt other launch files (e.g., `mapping_horizon.launch.py`) for different Livox models or if you prefer to use `loam_scanRegistration_horizon`. These should call the appropriate nodes (`livox_repub`, `loam_scanRegistration` or `loam_scanRegistration_horizon`, `loam_laserMapping`) and set necessary parameters like `N_SCANS` if `scanRegistration_horizon` is used.

### 7.2. RViz Visualization
The launch files typically start RViz with a pre-configured setup (e.g., `rviz_cfg/loam_livox.rviz`). You should see:
*   TF frames (`camera_init`, `aft_mapped`).
*   Published point clouds (map, features, registered scan).
*   Odometry path.

## 8. Rosbag Example (Updated for ROS 2)

The original README provided links to ROS 1 bag files. These can be converted to ROS 2 format or you can record new ROS 2 bags.

<div align="center"><img src="doc/results/mid40_hall_01.png" width=90% /></div>
<div align="center"><img src="doc/results/mid40_outdoor.png" width=90% /></div>
<!-- ... (keep other images and their respective sections) ... -->
<div align="center"><img src="doc/results/mid100_01.png" width=90% /></div>
<div align="center"><img src="doc/results/mid100_02.png" width=90% /></div>
<div align="center"><img src="doc/results/horizon_outdoor_01.png" width=90% /></div>
<div align="center"><img src="doc/results/horizon_parking_01.png" width=90% /></div>

**Example with a ROS 2 bag:**
1.  **Ensure your system is running:**
    ```bash
    # In one terminal (if not using a single comprehensive launch file for mapping):
    source ~/ros2_ws/install/setup.bash
    # ros2 run livox_mapping livox_repub_node (if needed and not in main launch)
    # ros2 run livox_mapping loam_scanRegistration_node 
    # ros2 run livox_mapping loam_laserMapping_node

    # Or, more simply, run your main launch file:
    ros2 launch livox_mapping mapping_mid.launch.py rviz:=true 
    ```
2.  **Play the ROS 2 Bag:**
    ```bash
    # In another terminal:
    source ~/ros2_ws/install/setup.bash
    ros2 bag play YOUR_ROS2_BAG_DIRECTORY 
    # Add --remap if topic names in the bag differ from those expected by the nodes.
    # e.g., ros2 bag play YOUR_ROS2_BAG_DIRECTORY --remap /livox_lidar_msg_from_bag:=/livox/lidar
    ```
    *(Links to original ROS 1 bags for reference: [mid40_hall_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_hall_example.bag), [mid40_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_outdoor.bag), [mid100_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid100_example.bag), [horizon_parking](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/horizon_parking.bag), [horizon_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/horizon_outdoor.bag))*


## 9. Important Notes & Known Issues
*   **PointType Consistency:** All nodes in the processing pipeline (`livox_repub`, `scanRegistration` or `scanRegistration_horizon`, `laserMapping`) must consistently use a PCL `PointType` that includes a `curvature` field (like `pcl::PointXYZINormal`) and correctly propagate the normalized timestamp information in this field. This has been addressed for `scanRegistration.cpp` and is correctly handled by `scanRegistration_horizon.cpp`.
*   **Curvature/Timestamp Normalization:** The `laserMapping` node assumes the `curvature` field it receives is `normalized_scan_time * 0.1`. The `livox_repub` node produces this format. If your data pipeline differs (e.g., if `scanRegistration` were to alter this scaling, which it currently does not after the fix), you must adjust the scaling factor in `laserMapping.cpp` (function `pointAssociateToMap_all`).
*   **TF Frames:** The transform `camera_init` -> `aft_mapped` published by `laserMapping` may need adjustments in its RPY mapping depending on your specific robot configuration and desired ROS coordinate frame conventions (e.g., REP-103/REP-105). Visual verification in RViz is crucial.

## 10. Acknowledgments
This package is based on the original LOAM algorithm by J. Zhang and S. Singh. We also acknowledge inspiration and reference from LOAM_NOTED.
*   LOAM: J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time. Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
*   [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED)

```
