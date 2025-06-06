# Livox Mapping (ROS 2)

Livox_mapping is a ROS 2 package for 3D SLAM using Livox LiDARs, based on the LOAM algorithm. It provides real-time odometry and mapping capabilities. This version has been adapted for ROS 2 and includes initial support for various Livox models including the Mid360.

<div align="center">
    <img src="doc/results/mid40_hall.gif" width = 45% >
    <img src="doc/results/horizon_parking.gif" width = 45% >
</div>

## Features (and Original Key Issues/Goals)
This package aims to provide robust LOAM-based SLAM for Livox Lidars.
*   Based on the well-known LOAM algorithm.
*   Supports various Livox LiDAR models, with specific configurations for Mid-series and Horizon, and initial support for Mid360.
*   Provides options for different feature extraction strategies.
*   Includes `livox_repub` node for converting Livox `CustomMsg` to `pcl::PointCloud<PointXYZINormal>` with timestamp information crucial for motion compensation.

Original goals from the ROS 1 version included:
1. Support for multiple Livox LiDAR models (ongoing with ROS 2).
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
    *   **Point Type:** Converts Livox points to `pcl::PointXYZI`. It populates:
        *   `intensity`: Encodes Livox line number (integer part) and reflectivity (decimal part).
        *   The `curvature` field is no longer populated with a timestamp. Timestamps are primarily handled by the message header.
*   **`scanRegistration` (Executable: `loam_scanRegistration`)**
    *   **Purpose:** Processes point clouds from `livox_repub` (or directly from a driver if configured) to extract geometric features: sharp edges (corners) and planar surfaces. This version is generally applicable to various Livox scan patterns.
    *   **Output:** Publishes feature clouds (`/laser_cloud_sharp`, `/laser_cloud_flat`).
    *   **Point Type:** Uses `pcl::PointXYZI`. It preserves `intensity` (line number + reflectivity) from the input cloud.
*   **`scanRegistration_horizon` (Executable: `loam_scanRegistration_horizon`)**
    *   **Purpose:** An alternative feature extraction node, potentially optimized for Livox Horizon or other multi-line Lidars where `N_SCANS` is a meaningful parameter.
    *   **Point Type:** Uses `pcl::PointXYZI` and preserves `intensity` from the input cloud.
*   **`laserMapping` (Executable: `loam_laserMapping`)**
    *   **Purpose:** Performs scan-to-map matching using the extracted features. It estimates the LiDAR's pose, and builds/updates a 3D map.
    *   **Point Type:** Expects input feature clouds with `pcl::PointXYZI` points. Motion distortion compensation that previously relied on a `curvature` field in `pointAssociateToMap_all` is currently not active (timestamps are per-message).

**Recommended Data Flow for Accurate Mapping:**
`livox_ros_driver2` -> `livox_repub` (outputs `/livox_pcl0` with `pcl::PointXYZI`) -> `loam_scanRegistration` OR `loam_scanRegistration_horizon` (consumes `/livox_pcl0`, outputs feature clouds with `pcl::PointXYZI`) -> `loam_laserMapping`.
The choice between `loam_scanRegistration` and `loam_scanRegistration_horizon` depends on the LiDAR model and scan pattern. `loam_scanRegistration` is more generic.

## 4. Parameters

Parameters can be set via launch files or command-line arguments.

### `livox_repub` Node
*   `to_merge_count` (int, default: `1` in C++ code): Number of input `livox_ros_driver2::msg::CustomMsg` messages to accumulate before publishing an aggregated point cloud. Can be set in its launch file if exposed.

### `scanRegistration_horizon` Node
*   `n_scans` (int, default: `6` in C++ code): Number of scan lines/rings assumed for feature extraction. This should be set according to your LiDAR characteristics or how `livox_repub` encodes line information into the intensity field.

### `laserMapping` Node (Example from `mapping_mid.launch.py`)
*   `map_file_path` (string, default: `" "`): Path to save the generated PCD map files. If left as an empty space or not specified, saving might be disabled or use an internal default.
*   `filter_parameter_corner` (double, default: `0.1` in `mapping_mid.launch.py`): Voxel grid leaf size for downsampling corner point clouds. (Node default is 0.2)
*   `filter_parameter_surf` (double, default: `0.2` in `mapping_mid.launch.py`): Voxel grid leaf size for downsampling surface point clouds. (Node default is 0.4)

*(Note: Default values for `laserMapping` parameters in C++ source may differ from launch files. Launch file values take precedence if set.)*

## 5. Topics & Messages

This section describes common topics. Actual topic names for feature clouds might vary if using `_horizon` versions.

### `livox_repub` Node
*   **Subscribes:**
    *   `/livox/lidar` (`livox_ros_driver2::msg::CustomMsg`): Raw Livox point data.
*   **Publishes:**
    *   `/livox_pcl0` (`sensor_msgs::msg::PointCloud2`): Processed cloud with `pcl::PointXYZINormal` points.

### `scanRegistration` / `scanRegistration_horizon` Node
*   **Subscribes:**
    *   `/livox_pcl0` (`sensor_msgs::msg::PointCloud2`): Input point cloud from `livox_repub`.
*   **Publishes:**
    *   `/livox_cloud` (or `/livox_cloud_horizon`) (`sensor_msgs::msg::PointCloud2`): Input cloud after some processing.
    *   `/laser_cloud_sharp` (or `/laser_cloud_sharp_horizon`) (`sensor_msgs::msg::PointCloud2`): Corner feature points.
    *   `/laser_cloud_flat` (or `/laser_cloud_flat_horizon`) (`sensor_msgs::msg::PointCloud2`): Surface feature points.

### `laserMapping` Node
*   **Subscribes:** (Topic names depend on which scanRegistration is used)
    *   `/laser_cloud_sharp` (or `/laser_cloud_sharp_horizon`)
    *   `/laser_cloud_flat` (or `/laser_cloud_flat_horizon`)
    *   `/livox_cloud` (or `/livox_cloud_horizon`)
*   **Publishes:**
    *   `/laser_cloud_surround` (`sensor_msgs::msg::PointCloud2`): Local map cloud.
    *   `/laser_cloud_surround_corner` (`sensor_msgs::msg::PointCloud2`): Corner points from local map.
    *   `/velodyne_cloud_registered` (`sensor_msgs::msg::PointCloud2`): Registered (deskewed) current scan.
    *   `/aft_mapped_to_init` (`nav_msgs::msg::Odometry`): LiDAR odometry in map frame.

## 6. Services
No ROS 2 services are explicitly defined by these nodes in the reviewed C++ code.

## 7. Usage (Running the Demo)

### 7.1. General Launching Strategy
1.  **Start `livox_ros_driver2`:** Use the appropriate launch file for your LiDAR model (e.g., Mid-40, Mid360, Horizon, Avia). This publishes the raw `livox_ros_driver2::msg::CustomMsg`.
    ```bash
    # Terminal 1: Source workspace, launch Livox driver
    source ~/ros2_ws/install/setup.bash
    ros2 launch livox_ros_driver2 <your_lidar_driver_launch.py> 
    ```
2.  **Start `livox_mapping`:** Use the corresponding launch file for your setup.
    ```bash
    # Terminal 2: Source workspace, launch livox_mapping
    source ~/ros2_ws/install/setup.bash
    ros2 launch livox_mapping <your_chosen_mapping_launch.launch.py> rviz:=true
    ```

### 7.1.1. For Livox Mid-40 / Mid-70 (using `mapping_mid.launch.py`)
This launch file uses `loam_scanRegistration` (generic feature extraction).
```bash
# Terminal 1 (Livox Driver - example for Mid-40):
# source ~/ros2_ws/install/setup.bash
# ros2 launch livox_ros_driver2 livox_lidar_launch.py # Adjust to your specific driver launch for Mid-40/70

# Terminal 2 (Livox Mapping):
source ~/ros2_ws/install/setup.bash
ros2 launch livox_mapping mapping_mid.launch.py rviz:=true
```

### 7.1.2. For Livox Mid360 (using `mapping_mid360.launch.py`)
This launch file starts `livox_repub`, then `loam_scanRegistration` (generic feature extraction), then `loam_laserMapping`.
```bash
# Terminal 1 (Livox Driver - example for Mid360):
# source ~/ros2_ws/install/setup.bash
# ros2 launch livox_ros_driver2 msg_MID360_launch.py # Or your specific Mid360 driver launch

# Terminal 2 (Livox Mapping):
source ~/ros2_ws/install/setup.bash
ros2 launch livox_mapping mapping_mid360.launch.py rviz:=true
```
**Note for Mid360:** The `loam_scanRegistration` node is used for its generality. Performance with Mid360's unique scan pattern should be verified. Parameter tuning in `laserMapping` (filter sizes) might be necessary.

### 7.1.3. For Livox Horizon (using `mapping_horizon_launch.py` - *needs conversion/creation*)
The original `mapping_horizon.launch` was XML. A `mapping_horizon.launch.py` would need to be created, likely launching `livox_repub`, `loam_scanRegistration_horizon` (setting the `n_scans` parameter appropriately for Horizon), and `loam_laserMapping`.

**Important Notes on Launching (General):**
*   Ensure the feature extraction pipeline consistently uses `pcl::PointXYZI`.
*   The `*_launch.py` files provided are examples. You may need to adapt them (e.g., to use `loam_scanRegistration_horizon` instead of `loam_scanRegistration` in `mapping_mid.launch.py` if preferred) or create new ones for different LiDARs or configurations.

### 7.2. RViz Visualization
The launch files typically start RViz with `rviz_cfg/loam_livox.rviz`. You should see:
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
1.  **Launch the mapping system:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    # Choose the appropriate launch file for your Lidar/bag data
    ros2 launch livox_mapping mapping_mid360.launch.py rviz:=true 
    ```
2.  **Play the ROS 2 Bag:**
    ```bash
    # In another terminal:
    source ~/ros2_ws/install/setup.bash
    ros2 bag play YOUR_ROS2_BAG_DIRECTORY 
    # Add --remap if topic names in the bag differ from those expected by the nodes.
    # e.g., ros2 bag play YOUR_ROS2_BAG_DIRECTORY --remap /livox_lidar_msg_from_bag:=/livox/lidar 
    # (The /livox/lidar topic is consumed by livox_repub_node)
    ```
    *(Links to original ROS 1 bags for reference: [mid40_hall_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_hall_example.bag), [mid40_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_outdoor.bag), [mid100_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid100_example.bag), [horizon_parking](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/horizon_parking.bag), [horizon_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/horizon_outdoor.bag))*


## 9. Important Notes & Known Issues
*   **PointType Consistency:** All nodes in the processing pipeline (`livox_repub`, `scanRegistration` or `scanRegistration_horizon`, `laserMapping`) now consistently use `pcl::PointXYZI`. The `curvature` field is no longer used for propagating normalized timestamp information. Timestamps are handled at the message level.
*   **Motion Distortion Compensation in `laserMapping`:** The `laserMapping` node's `pointAssociateToMap_all` function previously used the `curvature` field for motion deskewing within a scan. As this field is no longer available for that purpose, this specific per-point motion compensation is not currently active. Overall mapping relies on per-message timestamps.
*   **TF Frames:** The transform `camera_init` -> `aft_mapped` published by `laserMapping` may need adjustments in its RPY mapping depending on your specific robot configuration and desired ROS coordinate frame conventions (e.g., REP-103/REP-105). Visual verification in RViz is crucial.
*   **Livox Mid360:** The provided `mapping_mid360.launch.py` uses the generic `loam_scanRegistration` node. The performance and feature extraction quality with Mid360's unique scan pattern should be carefully evaluated. Parameters within `laserMapping` (e.g., filter sizes) or the choice of feature extractor might require tuning for optimal results with Mid360.

## 10. Acknowledgments
This package is based on the original LOAM algorithm by J. Zhang and S. Singh. We also acknowledge inspiration and reference from LOAM_NOTED.
*   LOAM: J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time. Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
*   [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED)

```
