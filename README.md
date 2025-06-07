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

## Pipeline Health Monitoring

This feature is designed to help diagnose potential issues within the SLAM pipeline by providing warnings when key operational metrics fall outside expected ranges. It works by logging `RCLCPP_WARN` messages when these metrics cross pre-configured thresholds.

A master switch, `health.enable_health_warnings` (boolean, default: `true`), is available for both the feature extraction (`scanRegistration`/`scanRegistration_horizon`) and `laserMapping` nodes. Setting this to `false` will suppress all health-related warnings from that specific node.

### Health Parameters for `scanRegistration` / `scanRegistration_horizon`

These parameters control warnings related to the feature extraction process. They are applicable to both `loam_scanRegistration` and `loam_scanRegistration_horizon` executables.

*   **`health.min_raw_points_for_feature_extraction`** (int, default: 10)
    *   **Purpose:** Minimum number of points required in the input cloud (after basic filtering like NaN removal) to proceed with feature extraction.
    *   **Warning Implication:** A warning indicates that the input cloud is too sparse. This could be due to issues with the LiDAR, driver, or `livox_repub` node, or an extremely sparse environment.
*   **`health.min_sharp_features`** (int, default: 15)
    *   **Purpose:** Minimum number of sharp (corner) features to be extracted.
    *   **Warning Implication:** Low sharp feature count might suggest a geometrically und-diverse environment (e.g., long corridors, open fields), or that the LiDAR data is not conducive to strong corner detection (e.g., noisy data, insufficient point density on edges).
*   **`health.min_flat_features`** (int, default: 40)
    *   **Purpose:** Minimum number of flat (surface) features to be extracted.
    *   **Warning Implication:** Similar to sharp features, a low count for flat features can indicate a lack of planar surfaces in the environment or issues with point cloud quality for surface fitting.

### Health Parameters for `laserMapping`

These parameters control warnings related to the scan-to-map matching and map update process in the `loam_laserMapping` node.

*   **`health.min_downsampled_corner_features`** (int, default: 12)
    *   **Purpose:** Minimum number of corner features from the current scan after voxel grid downsampling, before attempting ICP.
    *   **Warning Implication:** Very few corner features post-downsampling might lead to poor constraints for ICP, especially rotation.
*   **`health.min_downsampled_surf_features`** (int, default: 30)
    *   **Purpose:** Minimum number of surface features from the current scan after voxel grid downsampling.
    *   **Warning Implication:** Insufficient surface features can weaken the ICP solution, particularly for translation.
*   **`health.min_map_corner_points_for_icp`** (int, default: 30)
    *   **Purpose:** Minimum number of corner points retrieved from the local map to be used as target points for ICP.
    *   **Warning Implication:** If the local map doesn't have enough corner points in the vicinity of the current scan, ICP might be unreliable or skipped. This could indicate poor localization or an unexplored area.
*   **`health.min_map_surf_points_for_icp`** (int, default: 100)
    *   **Purpose:** Minimum number of surface points retrieved from the local map for ICP.
    *   **Warning Implication:** Similar to map corner points, a low count here suggests an insufficient local map for robust surface feature matching.
*   **`health.min_icp_correspondences`** (int, default: 40)
    *   **Purpose:** Minimum number of selected point correspondences (both corner and surface) used in the ICP optimization step.
    *   **Warning Implication:** Fewer correspondences than this threshold (but above the critical minimum of 50 which skips optimization) suggest a weak geometric link between the current scan and the map, potentially leading to less accurate pose updates.
*   **`health.max_icp_delta_rotation_deg`** (double, default: 5.0)
    *   **Purpose:** Maximum rotational correction (in degrees) applied by a single ICP iteration.
    *   **Warning Implication:** A very large rotational correction can indicate unstable tracking, a jump in localization, or issues with initial pose prediction.
*   **`health.max_icp_delta_translation_cm`** (double, default: 20.0)
    *   **Purpose:** Maximum translational correction (in centimeters) applied by a single ICP iteration.
    *   **Warning Implication:** Similar to large rotational corrections, significant translational jumps suggest instability or poor prior pose estimates.
*   **`health.warn_on_icp_degeneracy`** (bool, default: true)
    *   **Purpose:** Whether to log a warning if the ICP optimization encounters a degenerate geometry (e.g., trying to solve for translation along a corridor where only rotation is well-constrained).
    *   **Warning Implication:** Indicates that the current scan and map geometry do not provide enough constraints for a full 6-DOF pose update, potentially leading to drift in certain directions.

### Configuring Health Parameters in Launch Files

You can adjust these health monitoring thresholds via launch arguments in the Python launch files. For example, in `mapping_mid.launch.py` (or `mapping_horizon_launch.py`, `mapping_mid360_launch.py`, `mapping_outdoor_launch.py`):

1.  **Declare the launch argument:**
    ```python
    # In the launch file, e.g., mapping_mid.launch.py
    declare_sr_health_min_sharp_features_arg = DeclareLaunchArgument(
        'sr_health_min_sharp_features', default_value='20', # Default is 20
        description='Min sharp features in scanRegistration'
    )
    declare_lm_health_min_icp_correspondences_arg = DeclareLaunchArgument(
        'lm_health_min_icp_correspondences', default_value='75', # Default is 75
        description='Min ICP correspondences in laserMapping'
    )
    ```

2.  **Pass it to the node:**
    ```python
    # For scan_registration_node
    parameters=[
        {'health.min_sharp_features': LaunchConfiguration('sr_health_min_sharp_features')},
        # ... other sr params
    ]

    # For laser_mapping_node
    parameters=[
        # ... other lm params like markers_icp_corr
        {'health.min_icp_correspondences': LaunchConfiguration('lm_health_min_icp_correspondences')},
        # ... other lm health params
    ]
    ```

3.  **Add the declared argument to the `LaunchDescription` list:**
    ```python
    ld.add_action(declare_sr_health_min_sharp_features_arg)
    ld.add_action(declare_lm_health_min_icp_correspondences_arg)
    ```

4.  **Override from the command line when launching:**
    ```bash
    ros2 launch livox_mapping mapping_mid.launch.py sr_health_min_sharp_features:=15 lm_health_min_icp_correspondences:=60
    ```

### Example: Tuning for Specific Environments

Feedback from users, such as experiences in underground carparks, has highlighted scenarios where default feature detection thresholds might be too high. In such geometrically challenging environments (e.g., feature-poor, repetitive structures), if you observe frequent warnings about low feature counts (e.g., "Number of sharp features (X) is below threshold (Y)"), you might consider:
*   Reducing `sr_health_min_sharp_features` or `sr_health_min_flat_features`.
*   Subsequently, you might also need to adjust `lm_health_min_downsampled_corner_features`, `lm_health_min_downsampled_surf_features`, and `lm_health_min_icp_correspondences` if the input to `laserMapping` is consistently lower in features.
Lowering these thresholds can help the system continue tracking in sparse environments, but be mindful that it might also make the system more susceptible to incorrect matches if set too low. Always monitor the output odometry and map quality after tuning.

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

## Proposed Online Adaptive Parameter Adjustment Method

To enhance the robustness and performance of the LOAM pipeline under varying conditions, an online data-driven method for automatically adjusting pipeline parameters is proposed. This method is encapsulated by a conceptual `AdaptiveParameterManager` node.

### Purpose
The `AdaptiveParameterManager` aims to:
- Maintain healthy LOAM operation by dynamically tuning key parameters of the `laserMapping` node.
- Adapt to changing input data quality (e.g., feature-rich vs. feature-poor environments).
- Balance odometry performance and computational resource usage.

### Core Logic
The `AdaptiveParameterManager` would operate as follows:

1.  **Health Monitoring (Conceptual):**
    *   It would subscribe to health status messages published by the `scanRegistration` and `laserMapping` nodes. These messages would indicate states such as low feature counts, ICP instability (large corrections, degeneracy), or healthy operation.
    *   (Currently, `scanRegistration` and `laserMapping` only log warnings. They would need modification to publish structured health messages on dedicated ROS 2 topics).

2.  **Parameter Adjustment:**
    *   The primary parameters managed are `filter_parameter_corner` and `filter_parameter_surf` in the `laserMapping` node. These control the leaf sizes of voxel grid filters for downsampling corner and surface points.
    *   **If `laserMapping` reports too few features for ICP:** The manager reduces filter sizes to provide more points to the ICP algorithm.
    *   **If `laserMapping` reports ICP instability (e.g., large corrections, degeneracy):**
        *   If `scanRegistration` also reports few raw features (poor input), filter sizes in `laserMapping` are cautiously reduced.
        *   If `scanRegistration` is healthy (rich input), filter sizes are increased, as instability might be due to excessive/noisy points in dense environments.
    *   **If the system is healthy:** Filter sizes are slowly increased to probe for potential resource savings, as long as health remains good.
    *   **Health State Smoothing:** To prevent reactions to transient fluctuations, the manager now considers a health status from `scanRegistration` or `laserMapping` as "stabilized" only if it's reported consecutively for a defined number of times (currently hardcoded as `HEALTH_REPORT_STABILITY_THRESHOLD_ = 2`). Decisions are based on these stabilized states.
    *   **Cautious Healthy Probing:** When the overall system health becomes `HEALTHY`, the manager waits for a defined number of cycles (`PROBING_AFTER_N_HEALTHY_CYCLES_ = 2`) during which health remains stable before attempting to increase filter sizes (probing for resource savings). This ensures stability before making parameters more aggressive.

3.  **Simulated Resource Usage Feedback / Overload Detection:**
    *   To prevent the system from becoming unstable due to processing too many points (e.g., after reducing filter sizes too much), a cooldown mechanism is implemented:
        *   If parameter adjustments lead to a configurable number of consecutive ICP instability warnings, the system is considered "overloaded."
        *   During overload cooldown, filter sizes are temporarily increased to stabilize the pipeline, and further reductions are paused.
        *   The cooldown period is reset after a sustained period of healthy operation.

### Conceptual Integration with LOAM
-   **Health Publication:** `scanRegistration` and `laserMapping` nodes would need to be enhanced to publish their health status (defined in `adaptive_parameter_manager_types.h`) on new ROS 2 topics.
-   **Parameter Updates:** `AdaptiveParameterManager` would use a ROS 2 parameter client to dynamically set `filter_parameter_corner` and `filter_parameter_surf` on the `laserMapping` node. The `laserMapping` node would need to be modified to support dynamic parameter updates (e.g., using parameter callbacks) and apply these changes to its voxel grid filters.

### Current Status
-   The `AdaptiveParameterManager` class, along with its decision logic and health type definitions, has been designed and implemented.
-   The new files are:
    -   `src/adaptive_parameter_manager.h`
    -   `src/adaptive_parameter_manager.cpp`
    -   `src/adaptive_parameter_manager_types.h`
-   Full integration (modifying existing LOAM nodes for health publication and dynamic parameter handling) is a separate, subsequent task. This proposal focuses on the methodology and the standalone manager's logic.
-   The `AdaptiveParameterManager` internally uses a ROS 2 asynchronous parameter client for setting parameters on the `laserMapping` node. This ensures non-blocking behavior within the manager's own processing loop and resolves potential executor conflicts.

## 10. Acknowledgments
This package is based on the original LOAM algorithm by J. Zhang and S. Singh. We also acknowledge inspiration and reference from LOAM_NOTED.
*   LOAM: J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time. Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
*   [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED)

```
