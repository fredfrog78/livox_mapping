// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For tf2::toMsg
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp> // For cv::Mat, cv::solve, cv::eigen etc.
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include <vector>
#include <cmath>
#include <string>
#include <fstream> // For std::ofstream
#include <array>   // For std::array

typedef pcl::PointXYZI PointType;

// Utility functions (can be static members or in a utility namespace)
static double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}
// static double deg2rad(double degrees) { // Not used in provided snippet
//   return degrees * M_PI / 180.0;
// }

class LaserMapping : public rclcpp::Node {
public:
  LaserMapping() : Node("laser_mapping_node"),
                   laserCloudCenWidth_(10), // Default values from original code
                   laserCloudCenHeight_(5),
                   laserCloudCenDepth_(10),
                   laserCloudWidth_(21),
                   laserCloudHeight_(11),
                   laserCloudDepth_(21),
                   laserCloudNum_(laserCloudWidth_ * laserCloudHeight_ * laserCloudDepth_),
                   kfNum_(0),
                   filter_param_corner_(0.2), // Default values
                   filter_param_surf_(0.4) 
  {
    RCLCPP_INFO(this->get_logger(), "Initializing LaserMapping Node");

    // Declare and get parameters
    this->declare_parameter<bool>("markers_icp_corr", false);
    this->get_parameter("markers_icp_corr", markers_icp_corr_);
    RCLCPP_INFO(this->get_logger(), "ICP correspondence markers (markers_icp_corr): %s", markers_icp_corr_ ? "true" : "false");

    this->declare_parameter<bool>("markers_sel_features", false);
    this->get_parameter("markers_sel_features", markers_sel_features_);
    RCLCPP_INFO(this->get_logger(), "Selected feature markers (markers_sel_features): %s", markers_sel_features_ ? "true" : "false");

    this->declare_parameter<bool>("enable_icp_debug_logs", false);
    this->get_parameter("enable_icp_debug_logs", enable_icp_debug_logs_);
    RCLCPP_INFO(this->get_logger(), "Enable ICP debug console logs: %s", enable_icp_debug_logs_ ? "true" : "false");

    this->declare_parameter<std::string>("map_file_path", "map_data");
    this->get_parameter("map_file_path", map_file_path_);
    RCLCPP_INFO(this->get_logger(), "Map file path: %s", map_file_path_.c_str());

    this->declare_parameter<double>("filter_parameter_corner", 0.2);
    this->get_parameter("filter_parameter_corner", filter_param_corner_);
    RCLCPP_INFO(this->get_logger(), "Filter parameter corner: %f", filter_param_corner_);
    
    this->declare_parameter<double>("filter_parameter_surf", 0.4);
    this->get_parameter("filter_parameter_surf", filter_param_surf_);
    RCLCPP_INFO(this->get_logger(), "Filter parameter surf: %f", filter_param_surf_);

    RCLCPP_INFO(this->get_logger(), "--- LaserMapping Node Parameters ---");
    RCLCPP_INFO(this->get_logger(), "map_file_path: %s", map_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "filter_parameter_corner: %f", filter_param_corner_);
    RCLCPP_INFO(this->get_logger(), "filter_parameter_surf: %f", filter_param_surf_);

    // Declare and get health monitoring parameters
    this->declare_parameter<bool>("health.enable_health_warnings", true);
    this->declare_parameter<int>("health.min_downsampled_corner_features", 10);
    this->declare_parameter<int>("health.min_downsampled_surf_features", 30);
    this->declare_parameter<int>("health.min_map_corner_points_for_icp", 50);
    this->declare_parameter<int>("health.min_map_surf_points_for_icp", 200);
    this->declare_parameter<int>("health.min_icp_correspondences", 75);
    this->declare_parameter<double>("health.max_icp_delta_rotation_deg", 5.0);
    this->declare_parameter<double>("health.max_icp_delta_translation_cm", 20.0);
    this->declare_parameter<bool>("health.warn_on_icp_degeneracy", true);

    this->get_parameter("health.enable_health_warnings", enable_health_warnings_param_);
    this->get_parameter("health.min_downsampled_corner_features", min_downsampled_corner_features_param_);
    this->get_parameter("health.min_downsampled_surf_features", min_downsampled_surf_features_param_);
    this->get_parameter("health.min_map_corner_points_for_icp", min_map_corner_points_for_icp_param_);
    this->get_parameter("health.min_map_surf_points_for_icp", min_map_surf_points_for_icp_param_);
    this->get_parameter("health.min_icp_correspondences", min_icp_correspondences_param_);
    this->get_parameter("health.max_icp_delta_rotation_deg", max_icp_delta_rotation_deg_param_);
    this->get_parameter("health.max_icp_delta_translation_cm", max_icp_delta_translation_cm_param_);
    this->get_parameter("health.warn_on_icp_degeneracy", warn_on_icp_degeneracy_param_);

    RCLCPP_INFO(this->get_logger(), "Health Monitoring Parameters (LaserMapping):");
    RCLCPP_INFO(this->get_logger(), "  enable_health_warnings: %s", enable_health_warnings_param_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  min_downsampled_corner_features: %d", min_downsampled_corner_features_param_);
    RCLCPP_INFO(this->get_logger(), "  min_downsampled_surf_features: %d", min_downsampled_surf_features_param_);
    RCLCPP_INFO(this->get_logger(), "  min_map_corner_points_for_icp: %d", min_map_corner_points_for_icp_param_);
    RCLCPP_INFO(this->get_logger(), "  min_map_surf_points_for_icp: %d", min_map_surf_points_for_icp_param_);
    RCLCPP_INFO(this->get_logger(), "  min_icp_correspondences: %d", min_icp_correspondences_param_);
    RCLCPP_INFO(this->get_logger(), "  max_icp_delta_rotation_deg: %.2f", max_icp_delta_rotation_deg_param_);
    RCLCPP_INFO(this->get_logger(), "  max_icp_delta_translation_cm: %.2f", max_icp_delta_translation_cm_param_);
    RCLCPP_INFO(this->get_logger(), "  warn_on_icp_degeneracy: %s", warn_on_icp_degeneracy_param_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "------------------------------------");

    // Initialize PCL shared pointers
    laserCloudCornerLast_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudCornerLast_down_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfLast_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfLast_down_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudOri_ = std::make_shared<pcl::PointCloud<PointType>>();
    coeffSel_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurround2_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurround2_corner_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudCornerFromMap_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfFromMap_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudFullRes_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudFullRes2_ = std::make_shared<pcl::PointCloud<PointType>>(); // Used as a temporary copy
    laserCloudFullResColor_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    laserCloudFullResColor_pcd_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); // For saving final map

    kdtreeCornerFromMap_ = std::make_shared<pcl::KdTreeFLANN<PointType>>();
    kdtreeSurfFromMap_ = std::make_shared<pcl::KdTreeFLANN<PointType>>();
    
    // Initialize arrays of point clouds
    laserCloudCornerArray_.resize(laserCloudNum_);
    laserCloudSurfArray_.resize(laserCloudNum_);
    laserCloudCornerArray2_.resize(laserCloudNum_); // Used as temp in downsampling map cubes
    laserCloudSurfArray2_.resize(laserCloudNum_);  // Used as temp in downsampling map cubes
    for (int i = 0; i < laserCloudNum_; ++i) {
      laserCloudCornerArray_[i] = std::make_shared<pcl::PointCloud<PointType>>();
      laserCloudSurfArray_[i] = std::make_shared<pcl::PointCloud<PointType>>();
      laserCloudCornerArray2_[i] = std::make_shared<pcl::PointCloud<PointType>>();
      laserCloudSurfArray2_[i] = std::make_shared<pcl::PointCloud<PointType>>();
    }
    
    // Initialize transformation arrays
    transformTobeMapped_.fill(0.0f);
    transformAftMapped_.fill(0.0f);
    transformLastMapped_.fill(0.0f);

    // Initialize OpenCV Matrices
    matA0_ = cv::Mat(10, 3, CV_32F, cv::Scalar::all(0));
    matB0_ = cv::Mat(10, 1, CV_32F, cv::Scalar::all(-1));
    matX0_ = cv::Mat(10, 1, CV_32F, cv::Scalar::all(0)); // Should be 3x1 for plane params
    matA1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1_ = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    matV1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matP_ = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));


    // Initialize voxel grid filters
    downSizeFilterCorner_.setLeafSize(filter_param_corner_, filter_param_corner_, filter_param_corner_);
    downSizeFilterSurf_.setLeafSize(filter_param_surf_, filter_param_surf_, filter_param_surf_);

    // Publishers
    pubLaserCloudSurround_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround", 100);
    pubLaserCloudSurroundCorner_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround_corner", 100);
    pubLaserCloudFullRes_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered", 100);
    pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100);
    pub_icp_correspondence_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/icp_correspondences", 10);
    pub_selected_feature_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/selected_features", 10);

    odomAftMapped_.header.frame_id = "camera_init"; // camera_init / map
    odomAftMapped_.child_frame_id = "aft_mapped";    // aft_mapped / body

    // Subscribers
    subLaserCloudCornerLast_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laser_cloud_sharp", rclcpp::QoS(100), std::bind(&LaserMapping::laserCloudCornerLastHandler, this, std::placeholders::_1));
    subLaserCloudSurfLast_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laser_cloud_flat", rclcpp::QoS(100), std::bind(&LaserMapping::laserCloudSurfLastHandler, this, std::placeholders::_1));
    subLaserCloudFullRes_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox_cloud", rclcpp::QoS(100), std::bind(&LaserMapping::laserCloudFullResHandler, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "--- LaserMapping Subscribed Topics ---");
    if (subLaserCloudCornerLast_) {
        RCLCPP_INFO(this->get_logger(), "Subscribed to corner points on topic: %s", subLaserCloudCornerLast_->get_topic_name());
    }
    if (subLaserCloudSurfLast_) {
        RCLCPP_INFO(this->get_logger(), "Subscribed to surface points on topic: %s", subLaserCloudSurfLast_->get_topic_name());
    }
    if (subLaserCloudFullRes_) {
        RCLCPP_INFO(this->get_logger(), "Subscribed to full resolution cloud on topic: %s", subLaserCloudFullRes_->get_topic_name());
    }
    RCLCPP_INFO(this->get_logger(), "------------------------------------");

    // TF Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Timer for main processing loop
    // The original code uses ros::Rate(100) and checks conditions.
    // A timer can mimic this, or we can rely on callbacks setting flags and spin_some.
    // For a closer port of the loop structure, a timer is suitable.
    // Let's make it 10ms (100Hz)
    processing_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&LaserMapping::processMappingLoop, this));

    RCLCPP_INFO(this->get_logger(), "LaserMapping Node Initialized.");
  }

  ~LaserMapping() {
    RCLCPP_INFO(this->get_logger(), "LaserMapping Node Shutting down. Saving map...");
    saveMap();
  }


private:
  // Member Variables
  double timeLaserCloudCornerLast_, timeLaserCloudSurfLast_, timeLaserCloudFullRes_;
  bool newLaserCloudCornerLast_ = false, newLaserCloudSurfLast_ = false, newLaserCloudFullRes_ = false;

  int laserCloudCenWidth_, laserCloudCenHeight_, laserCloudCenDepth_;
  const int laserCloudWidth_, laserCloudHeight_, laserCloudDepth_; // Size of the map cube grid
  const int laserCloudNum_; // Total number of cubes in the map grid

  std::vector<int> laserCloudValidInd_; // Max size 125 based on original code
  std::vector<int> laserCloudSurroundInd_; // Max size 125
  int laserCloudValidNum_ = 0;
  int laserCloudSurroundNum_ = 0;

  // PCL Point Clouds
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_, laserCloudCornerLast_down_;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_, laserCloudSurfLast_down_;
  pcl::PointCloud<PointType>::Ptr laserCloudOri_, coeffSel_;
  pcl::PointCloud<PointType>::Ptr laserCloudSurround2_, laserCloudSurround2_corner_;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_, laserCloudSurfFromMap_;
  pcl::PointCloud<PointType>::Ptr laserCloudFullRes_, laserCloudFullRes2_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_, laserCloudFullResColor_pcd_;

  // Arrays of Point Clouds for map cubes
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray_;
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray_;
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray2_; // Temp for downsampling
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray2_;  // Temp for downsampling

  // KD-Trees
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap_, kdtreeSurfFromMap_;

  // Transformations (rx, ry, rz, tx, ty, tz)
  std::array<float, 6> transformTobeMapped_;
  std::array<float, 6> transformAftMapped_;
  std::array<float, 6> transformLastMapped_;
  
  // Keyframe poses for saving map data
  std::vector<Eigen::Matrix<float,7,1>> keyframe_pose_; // [qx,qy,qz,qw,tx,ty,tz]
  // std::vector<Eigen::Matrix4f> pose_map_; // Not obviously used in the snippet, might be for a different feature

  int kfNum_; // Keyframe counter

  // OpenCV Matrices for ICP solving
  cv::Mat matA0_, matB0_, matX0_; // For plane fitting
  cv::Mat matA1_, matD1_, matV1_; // For PCA on corner points
  cv::Mat matP_; // For degenaracy handling

  bool isDegenerate_ = false;

  // Voxel Grid Filters
  pcl::VoxelGrid<PointType> downSizeFilterCorner_;
  pcl::VoxelGrid<PointType> downSizeFilterSurf_;
  // pcl::VoxelGrid<PointType> downSizeFilterFull; // Not used in snippet

  // ROS Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurroundCorner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_icp_correspondence_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_selected_feature_markers_;
  
  nav_msgs::msg::Odometry odomAftMapped_;

  // ROS Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudCornerLast_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudSurfLast_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes_;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer for the main processing loop
  rclcpp::TimerBase::SharedPtr processing_timer_;

  // Parameters
  std::string map_file_path_;
  double filter_param_corner_;
  double filter_param_surf_;
  bool markers_icp_corr_;
  bool markers_sel_features_;
  bool enable_icp_debug_logs_;

  // Health monitoring parameters
  bool enable_health_warnings_param_;
  int min_downsampled_corner_features_param_;
  int min_downsampled_surf_features_param_;
  int min_map_corner_points_for_icp_param_;
  int min_map_surf_points_for_icp_param_;
  int min_icp_correspondences_param_;
  double max_icp_delta_rotation_deg_param_;
  double max_icp_delta_translation_cm_param_;
  bool warn_on_icp_degeneracy_param_;

  // Helper: Convert transform array to Eigen::Matrix4f
  Eigen::Matrix4f trans_euler_to_matrix(const std::array<float, 6>& trans) {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      Eigen::Matrix3f R;
      // Original LOAM RPY order seems to be ZXY (yaw, then pitch, then roll)
      // Eigen::AngleAxisf rollAngle(trans[0], Eigen::Vector3f::UnitX());
      // Eigen::AngleAxisf pitchAngle(trans[1], Eigen::Vector3f::UnitY());
      // Eigen::AngleAxisf yawAngle(trans[2], Eigen::Vector3f::UnitZ());
      // R = yawAngle * pitchAngle * rollAngle; // Check this order carefully
      
      // The pointAssociateToMap uses Rz * Rx * Ry
      // transformTobeMapped[0] -> rx
      // transformTobeMapped[1] -> ry
      // transformTobeMapped[2] -> rz
      // So, R = Ry * Rx * Rz based on pointAssociateToMap
      Eigen::AngleAxisf rollAngle(trans[0], Eigen::Vector3f::UnitX());    // rx
      Eigen::AngleAxisf pitchAngle(trans[1], Eigen::Vector3f::UnitY());   // ry
      Eigen::AngleAxisf yawAngle(trans[2], Eigen::Vector3f::UnitZ());     // rz
      
      // Original code's pointAssociateToMap implies: R_final = R_y * R_x * R_z
      R = pitchAngle * rollAngle * yawAngle; // This is Ry * Rx * Rz
      
      T.block<3,3>(0,0) = R;
      T.block<3,1>(0,3) = Eigen::Vector3f(trans[3], trans[4], trans[5]);
      return T;
  }

  void transformAssociateToMap() {
    // Predict current transform based on last two frames
    Eigen::Matrix4f T_aft = trans_euler_to_matrix(transformAftMapped_);
    Eigen::Matrix4f T_last = trans_euler_to_matrix(transformLastMapped_);
    Eigen::Matrix4f T_predict = T_aft * (T_last.inverse() * T_aft); // Extrapolate motion

    Eigen::Matrix3f R_predict = T_predict.block<3,3>(0,0);
    // Convert rotation matrix back to RPY angles (rx, ry, rz)
    // The order YXZ (pitch, roll, yaw) for .eulerAngles(1,0,2) might be what original LOAM uses for storing angles
    // Need to verify which euler angle convention is used for transformTobeMapped_[0,1,2]
    // Based on pointAssociateToMap: 0->X, 1->Y, 2->Z rotations.
    // If R = Ry * Rx * Rz, then eulerAngles(1,0,2) means Y,X,Z.
    // So, euler_predict[0] = ry, euler_predict[1] = rx, euler_predict[2] = rz
    // This seems to match: transformTobeMapped[0] = rx, [1] = ry, [2] = rz
    Eigen::Vector3f euler_predict_angles = R_predict.eulerAngles(1,0,2); // Get Y, X, Z angles

    transformTobeMapped_[0] = euler_predict_angles[1]; // rx
    transformTobeMapped_[1] = euler_predict_angles[0]; // ry
    transformTobeMapped_[2] = euler_predict_angles[2]; // rz
    transformTobeMapped_[3] = T_predict.coeff(0,3);
    transformTobeMapped_[4] = T_predict.coeff(1,3);
    transformTobeMapped_[5] = T_predict.coeff(2,3);

    // RCLCPP_DEBUG(this->get_logger(), "transformAftMapped: %f %f %f %f %f %f",
    //     transformAftMapped_[0], transformAftMapped_[1], transformAftMapped_[2],
    //     transformAftMapped_[3], transformAftMapped_[4], transformAftMapped_[5]);
    // RCLCPP_DEBUG(this->get_logger(), "transformTobeMapped (predicted): %f %f %f %f %f %f",
    //     transformTobeMapped_[0], transformTobeMapped_[1], transformTobeMapped_[2],
    //     transformTobeMapped_[3], transformTobeMapped_[4], transformTobeMapped_[5]);
  }

  void transformUpdate() {
    transformLastMapped_ = transformAftMapped_;
    transformAftMapped_ = transformTobeMapped_;
  }

  void pointAssociateToMap(PointType const * const pi, PointType * const po) {
    float rx = transformTobeMapped_[0];
    float ry = transformTobeMapped_[1];
    float rz = transformTobeMapped_[2];
    float tx = transformTobeMapped_[3];
    float ty = transformTobeMapped_[4];
    float tz = transformTobeMapped_[5];

    // Rz
    float x1 = cos(rz) * pi->x - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x + cos(rz) * pi->y;
    float z1 = pi->z;
    // Rx
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;
    // Ry
    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    po->intensity = pi->intensity;
  }
  
  // This function was in original code, seems to use intensity as interpolation factor 's'
  // For Livox, intensity is line_num.decimal_reflectivity, curvature is timestamp_offset
  // If this 's' is for motion deskewing within a scan, it should use curvature (timestamp)
  void pointAssociateToMap_all(PointType const * const pi, PointType * const po) {
    // Assuming pi->curvature holds the normalized scan time (0 to 1)
    // Original code used: s = pi->intensity - int(pi->intensity);
    // For Livox, curvature has this role.
    float s = 0.0f; // FIXME: Curvature/normalized_time not available from PointXYZI. Set to 0.0 for now.
                             // scanRegistration_horizon uses point.curvature = s*0.1
                             // So, s = curvature / 0.1 if that's the input.
                             // Assuming it's already 0-1 from the feature extraction node.

    float rx = (1 - s) * transformLastMapped_[0] + s * transformAftMapped_[0];
    float ry = (1 - s) * transformLastMapped_[1] + s * transformAftMapped_[1];
    float rz = (1 - s) * transformLastMapped_[2] + s * transformAftMapped_[2];
    float tx = (1 - s) * transformLastMapped_[3] + s * transformAftMapped_[3];
    float ty = (1 - s) * transformLastMapped_[4] + s * transformAftMapped_[4];
    float tz = (1 - s) * transformLastMapped_[5] + s * transformAftMapped_[5];

    float x1 = cos(rz) * pi->x - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x + cos(rz) * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    po->intensity = pi->intensity; // Keep original intensity (line + reflectivity)
    // po->curvature = pi->curvature; // Keep original curvature (timestamp)
  }


  void RGBpointAssociateToMap(PointType const * const pi, pcl::PointXYZRGB * const po) {
    // Using transformAftMapped_ as this is for coloring the final point cloud at estimated pose
    float rx = transformAftMapped_[0];
    float ry = transformAftMapped_[1];
    float rz = transformAftMapped_[2];
    float tx = transformAftMapped_[3];
    float ty = transformAftMapped_[4];
    float tz = transformAftMapped_[5];

    float x1 = cos(rz) * pi->x - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x + cos(rz) * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;

    // Color mapping based on intensity (reflectivity part)
    float intensity_val = pi->intensity; // This should be line_num.reflectivity
    float reflectivity = (intensity_val - std::floor(intensity_val)) * 10000.0f; // Extract decimal part and scale

    if (reflectivity < 30) {
        po->r = 0; po->g = static_cast<uint8_t>((reflectivity * 255 / 30)); po->b = 255;
    } else if (reflectivity < 90) {
        po->r = 0; po->g = 255; po->b = static_cast<uint8_t>(((90 - reflectivity) * 255) / 60);
    } else if (reflectivity < 150) {
        po->r = static_cast<uint8_t>(((reflectivity - 90) * 255) / 60); po->g = 255; po->b = 0;
    } else {
        po->r = 255; po->g = static_cast<uint8_t>(((255 - reflectivity) * 255) / (255 - 150)); po->b = 0;
    }
  }

  // Callbacks
  void laserCloudCornerLastHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PointCloud2 message on topic %s. Skipping conversion.", subLaserCloudCornerLast_->get_topic_name());
      return;
    }
    timeLaserCloudCornerLast_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    laserCloudCornerLast_->clear();
    pcl::fromROSMsg(*msg, *laserCloudCornerLast_);
    newLaserCloudCornerLast_ = true;
  }

  void laserCloudSurfLastHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PointCloud2 message on topic %s. Skipping conversion.", subLaserCloudSurfLast_->get_topic_name());
      return;
    }
    timeLaserCloudSurfLast_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    laserCloudSurfLast_->clear();
    pcl::fromROSMsg(*msg, *laserCloudSurfLast_);
    newLaserCloudSurfLast_ = true;
  }

  void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PointCloud2 message on topic %s. Skipping conversion.", subLaserCloudFullRes_->get_topic_name());
      return;
    }
    timeLaserCloudFullRes_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    laserCloudFullRes_->clear();
    // laserCloudFullResColor_->clear(); // Cleared before use in loop
    pcl::fromROSMsg(*msg, *laserCloudFullRes_);
    newLaserCloudFullRes_ = true;
  }
  
  void saveMap() {
    if (!rclcpp::ok()) { // If shutdown is initiated, try to save.
        RCLCPP_WARN(this->get_logger(), "Shutdown detected, attempting to save map files.");
    }

    std::string surf_filename(map_file_path_ + "/surf.pcd");
    std::string corner_filename(map_file_path_ + "/corner.pcd");
    std::string all_points_filename(map_file_path_ + "/all_points.pcd");
    std::ofstream keyframe_file(map_file_path_ + "/key_frame.txt");

    if (keyframe_file.is_open()) {
        for(const auto& kf : keyframe_pose_){
            keyframe_file << kf[0] << " "<< kf[1] << " "<< kf[2] << " "<< kf[3] << " "
                          << kf[4] << " "<< kf[5] << " "<< kf[6] << std::endl;
        }
        keyframe_file.close();
        RCLCPP_INFO(this->get_logger(), "Keyframe poses saved to %s", (map_file_path_ + "/key_frame.txt").c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open keyframe_file for writing: %s", (map_file_path_ + "/key_frame.txt").c_str());
    }
    
    // Concatenate all map cubes for saving
    pcl::PointCloud<PointType>::Ptr mapSurfTotal(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr mapCornerTotal(new pcl::PointCloud<PointType>());
    for(int i=0; i<laserCloudNum_; ++i) {
        *mapSurfTotal += *laserCloudSurfArray_[i];
        *mapCornerTotal += *laserCloudCornerArray_[i];
    }


    pcl::PCDWriter pcd_writer;
    if (mapSurfTotal->size() > 0) {
        pcd_writer.writeBinary(surf_filename, *mapSurfTotal);
        RCLCPP_INFO(this->get_logger(), "Surface map saved to %s (%ld points)", surf_filename.c_str(), mapSurfTotal->size());
    } else {
        RCLCPP_WARN(this->get_logger(), "No surface points to save.");
    }

    if (mapCornerTotal->size() > 0) {
        pcd_writer.writeBinary(corner_filename, *mapCornerTotal);
        RCLCPP_INFO(this->get_logger(), "Corner map saved to %s (%ld points)", corner_filename.c_str(), mapCornerTotal->size());
    } else {
        RCLCPP_WARN(this->get_logger(), "No corner points to save.");
    }
    
    if (laserCloudFullResColor_pcd_->size() > 0) {
        pcd_writer.writeBinary(all_points_filename, *laserCloudFullResColor_pcd_);
        RCLCPP_INFO(this->get_logger(), "Full resolution colored map saved to %s (%ld points)", all_points_filename.c_str(), laserCloudFullResColor_pcd_->size());
    } else {
        RCLCPP_WARN(this->get_logger(), "No full resolution colored points to save.");
    }
  }


  // Main processing loop, called by a timer
  void processMappingLoop() {
    if (!(newLaserCloudCornerLast_ && newLaserCloudSurfLast_ && newLaserCloudFullRes_ &&
        std::abs(timeLaserCloudSurfLast_ - timeLaserCloudCornerLast_) < 0.005 && // Tolerance of 5ms
        std::abs(timeLaserCloudFullRes_ - timeLaserCloudCornerLast_) < 0.005)) {
      return; // Wait for all messages
    }
    // RCLCPP_INFO(this->get_logger(), "Processing new frame...");

    // clock_t t1,t2,t3,t4; // For timing, use rclcpp::Clock for ROS2 style timing if needed for logs
    // t1 = clock();

    newLaserCloudCornerLast_ = false;
    newLaserCloudSurfLast_ = false;
    newLaserCloudFullRes_ = false;

    // Initial prediction of transformation
    // If it's the first frame, transformAftMapped and LastMapped are zero, so TobeMapped is zero.
    // Otherwise, predict based on previous motion.
    // This was commented out in original main loop, but seems important.
    // For LOAM, the first scan is usually added to map directly without ICP.
    // This logic depends on whether odometry is available or if it's the first scan.
    // The original code did not explicitly handle the "first scan" case in this part of the loop,
    // but transformAssociateToMap() would result in transformTobeMapped being zeros if previous transforms were zero.
    transformAssociateToMap();


    PointType pointOnYAxis; // Used for FOV checks
    pointOnYAxis.x = 0.0;
    pointOnYAxis.y = 10.0; // Arbitrary point far on Y-axis
    pointOnYAxis.z = 0.0;
    pointAssociateToMap(&pointOnYAxis, &pointOnYAxis); // Transform this point to map frame

    // Calculate current center cube indices based on predicted pose
    int centerCubeI = static_cast<int>((transformTobeMapped_[3] + 25.0) / 50.0) + laserCloudCenWidth_;
    int centerCubeJ = static_cast<int>((transformTobeMapped_[4] + 25.0) / 50.0) + laserCloudCenHeight_;
    int centerCubeK = static_cast<int>((transformTobeMapped_[5] + 25.0) / 50.0) + laserCloudCenDepth_;

    // Adjust for negative coordinates
    if (transformTobeMapped_[3] + 25.0 < 0) centerCubeI--;
    if (transformTobeMapped_[4] + 25.0 < 0) centerCubeJ--;
    if (transformTobeMapped_[5] + 25.0 < 0) centerCubeK--;

    // Shift map cubes if current pose is near the edge of the loaded map area
    // X-axis
    while (centerCubeI < 3) {
        for (int j = 0; j < laserCloudHeight_; j++) {
            for (int k = 0; k < laserCloudDepth_; k++) {
                // Move data from right to left
                pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                for (int i = laserCloudWidth_ - 1; i >= 1; i--) {
                    laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudCornerArray_[i - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                    laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudSurfArray_[i - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                }
                laserCloudCornerArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
                laserCloudSurfArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
                tempCorner->clear(); tempSurf->clear();
            }
        }
        centerCubeI++; laserCloudCenWidth_++;
    }
    while (centerCubeI >= laserCloudWidth_ - 3) {
        for (int j = 0; j < laserCloudHeight_; j++) {
            for (int k = 0; k < laserCloudDepth_; k++) {
                pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                for (int i = 0; i < laserCloudWidth_ - 1; i++) {
                    laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudCornerArray_[i + 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                    laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudSurfArray_[i + 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
                }
                laserCloudCornerArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
                laserCloudSurfArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
                tempCorner->clear(); tempSurf->clear();
            }
        }
        centerCubeI--; laserCloudCenWidth_--;
    }
    // Y-axis shift (similar logic for J index)
    while (centerCubeJ < 3) {
        for (int i = 0; i < laserCloudWidth_; i++) {
            for (int k = 0; k < laserCloudDepth_; k++) {
                 pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * (laserCloudHeight_ - 1) + laserCloudWidth_ * laserCloudHeight_ * k];
                 pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * (laserCloudHeight_ - 1) + laserCloudWidth_ * laserCloudHeight_ * k];
                for (int j = laserCloudHeight_ - 1; j >= 1; j--) {
                    laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudCornerArray_[i + laserCloudWidth_ * (j-1) + laserCloudWidth_ * laserCloudHeight_ * k];
                    laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudSurfArray_[i + laserCloudWidth_ * (j-1) + laserCloudWidth_ * laserCloudHeight_ * k];
                }
                laserCloudCornerArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
                laserCloudSurfArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
                tempCorner->clear(); tempSurf->clear();
            }
        }
        centerCubeJ++; laserCloudCenHeight_++;
    }
    while (centerCubeJ >= laserCloudHeight_ - 3) {
         for (int i = 0; i < laserCloudWidth_; i++) {
            for (int k = 0; k < laserCloudDepth_; k++) {
                pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k];
                pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k];
                for (int j = 0; j < laserCloudHeight_ - 1; j++) {
                     laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudCornerArray_[i + laserCloudWidth_ * (j+1) + laserCloudWidth_ * laserCloudHeight_ * k];
                    laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudSurfArray_[i + laserCloudWidth_ * (j+1) + laserCloudWidth_ * laserCloudHeight_ * k];
                }
                laserCloudCornerArray_[i + laserCloudWidth_ * (laserCloudHeight_-1) + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
                laserCloudSurfArray_[i + laserCloudWidth_ * (laserCloudHeight_-1) + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
                tempCorner->clear(); tempSurf->clear();
            }
        }
        centerCubeJ--; laserCloudCenHeight_--;
    }
    // Z-axis shift (similar logic for K index)
     while (centerCubeK < 3) {
        for (int i = 0; i < laserCloudWidth_; i++) {
            for (int j = 0; j < laserCloudHeight_; j++) {
                pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_ - 1)];
                pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_ - 1)];
                for (int k = laserCloudDepth_ - 1; k >= 1; k--) {
                    laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k-1)];
                    laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k-1)];
                }
                laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0] = tempCorner;
                laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0] = tempSurf;
                tempCorner->clear(); tempSurf->clear();
            }
        }
        centerCubeK++; laserCloudCenDepth_++;
    }
    while (centerCubeK >= laserCloudDepth_ - 3) {
        for (int i = 0; i < laserCloudWidth_; i++) {
            for (int j = 0; j < laserCloudHeight_; j++) {
                pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0];
                pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0];
                for (int k = 0; k < laserCloudDepth_ - 1; k++) {
                    laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k+1)];
                    laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] =
                        laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k+1)];
                }
                laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_-1)] = tempCorner;
                laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_-1)] = tempSurf;
                tempCorner->clear(); tempSurf->clear();
            }
        }
        centerCubeK--; laserCloudCenDepth_--;
    }

    // Select map cubes to match against
    int laserCloudValidNum_ = 0;
    int laserCloudSurroundNum_ = 0;
    laserCloudValidInd_.assign(125, 0); // Max 5x5x5 cubes = 125
    laserCloudSurroundInd_.assign(125, 0);


    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
        for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
          if (i >= 0 && i < laserCloudWidth_ &&
              j >= 0 && j < laserCloudHeight_ &&
              k >= 0 && k < laserCloudDepth_) {
            
            float centerX = 50.0f * (i - laserCloudCenWidth_);
            float centerY = 50.0f * (j - laserCloudCenHeight_);
            float centerZ = 50.0f * (k - laserCloudCenDepth_);

            // Check if cube is within FOV (simplified FOV check)
            bool isInLaserFOV = false;
            for (int ii = -1; ii <= 1; ii += 2) {
              for (int jj = -1; jj <= 1; jj += 2) {
                for (int kk = -1; kk <= 1; kk += 2) {
                  float cornerX = centerX + 25.0f * ii;
                  float cornerY = centerY + 25.0f * jj;
                  float cornerZ = centerZ + 25.0f * kk;

                  float squaredSide1 = (transformTobeMapped_[3] - cornerX) * (transformTobeMapped_[3] - cornerX) +
                                       (transformTobeMapped_[4] - cornerY) * (transformTobeMapped_[4] - cornerY) +
                                       (transformTobeMapped_[5] - cornerZ) * (transformTobeMapped_[5] - cornerZ);
                  float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) +
                                       (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY) +
                                       (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);
                  
                  // Original check, related to dot products and angles for FOV
                  float check1 = 100.0f + squaredSide1 - squaredSide2 - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);
                  float check2 = 100.0f + squaredSide1 - squaredSide2 + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);
                  if (check1 < 0 && check2 > 0) {
                    isInLaserFOV = true;
                    break; 
                  }
                }
                if(isInLaserFOV) break;
              }
              if(isInLaserFOV) break;
            }

            int cubeIdx = i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k;
            if (isInLaserFOV && laserCloudValidNum_ < 125) { // Ensure not to exceed bounds
              laserCloudValidInd_[laserCloudValidNum_++] = cubeIdx;
            }
            if (laserCloudSurroundNum_ < 125) { // Ensure not to exceed bounds
                 laserCloudSurroundInd_[laserCloudSurroundNum_++] = cubeIdx;
            }
          }
        }
      }
    }

    laserCloudCornerFromMap_->clear();
    laserCloudSurfFromMap_->clear();
    for (int i = 0; i < laserCloudValidNum_; i++) {
      *laserCloudCornerFromMap_ += *laserCloudCornerArray_[laserCloudValidInd_[i]];
      *laserCloudSurfFromMap_ += *laserCloudSurfArray_[laserCloudValidInd_[i]];
    }
    // int laserCloudCornerFromMapNum = laserCloudCornerFromMap_->points.size();
    // int laserCloudSurfFromMapNum = laserCloudSurfFromMap_->points.size();

    // Downsample current scan features
    laserCloudCornerLast_down_->clear();
    downSizeFilterCorner_.setInputCloud(laserCloudCornerLast_);
    downSizeFilterCorner_.filter(*laserCloudCornerLast_down_);
    // int laserCloudCornerLast_downNum = laserCloudCornerLast_down_->points.size();

    laserCloudSurfLast_down_->clear();
    downSizeFilterSurf_.setInputCloud(laserCloudSurfLast_);
    downSizeFilterSurf_.filter(*laserCloudSurfLast_down_);
    // int laserCloudSurfLast_downNum = laserCloudSurfLast_down_->points.size();

    if (enable_health_warnings_param_) {
        if (static_cast<int>(laserCloudCornerLast_down_->size()) < min_downsampled_corner_features_param_) {
            RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of downsampled corner features (%ld) is below threshold (%d).", laserCloudCornerLast_down_->size(), min_downsampled_corner_features_param_);
        }
        if (static_cast<int>(laserCloudSurfLast_down_->size()) < min_downsampled_surf_features_param_) {
            RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of downsampled surf features (%ld) is below threshold (%d).", laserCloudSurfLast_down_->size(), min_downsampled_surf_features_param_);
        }
    }

    // t2 = clock();

    // Main ICP optimization loop
    if (enable_icp_debug_logs_) {
      RCLCPP_INFO(this->get_logger(), "Pre-ICP: laserCloudCornerLast_down size: %ld, laserCloudSurfLast_down size: %ld", laserCloudCornerLast_down_->size(), laserCloudSurfLast_down_->size());
    }

    if (enable_health_warnings_param_) {
        if (static_cast<int>(laserCloudCornerFromMap_->points.size()) < min_map_corner_points_for_icp_param_) {
            RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of map corner points for ICP (%ld) is below threshold (%d).", laserCloudCornerFromMap_->points.size(), min_map_corner_points_for_icp_param_);
        }
        if (static_cast<int>(laserCloudSurfFromMap_->points.size()) < min_map_surf_points_for_icp_param_) {
            RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of map surf points for ICP (%ld) is below threshold (%d).", laserCloudSurfFromMap_->points.size(), min_map_surf_points_for_icp_param_);
        }
    }

    if (laserCloudCornerFromMap_->points.size() > 10 && laserCloudSurfFromMap_->points.size() > 100) { // Existing critical check to skip ICP
      if (enable_icp_debug_logs_) {
        RCLCPP_INFO(this->get_logger(), "ICP Start: laserCloudCornerFromMap size: %ld, laserCloudSurfFromMap size: %ld", laserCloudCornerFromMap_->size(), laserCloudSurfFromMap_->size());
      }
      kdtreeCornerFromMap_->setInputCloud(laserCloudCornerFromMap_);
      kdtreeSurfFromMap_->setInputCloud(laserCloudSurfFromMap_);

      visualization_msgs::msg::MarkerArray correspondence_marker_array;
      visualization_msgs::msg::Marker map_corner_pts_marker, scan_corner_pts_marker, corner_lines_marker;
      visualization_msgs::msg::Marker map_surface_pts_marker, scan_surface_pts_marker, surface_lines_marker;
      visualization_msgs::msg::Marker selected_points_marker; // Keep for selected features if that logic remains separate

      if (markers_icp_corr_) {
        // Initialization for Map Corner Points
        map_corner_pts_marker.header.frame_id = "camera_init";
        map_corner_pts_marker.ns = "map_corner_pts";
        map_corner_pts_marker.id = 0;
        map_corner_pts_marker.type = visualization_msgs::msg::Marker::POINTS;
        map_corner_pts_marker.action = visualization_msgs::msg::Marker::ADD;
        map_corner_pts_marker.pose.orientation.w = 1.0;
        map_corner_pts_marker.scale.x = 0.05; map_corner_pts_marker.scale.y = 0.05; map_corner_pts_marker.scale.z = 0.05;
        map_corner_pts_marker.color.r = 1.0f; map_corner_pts_marker.color.g = 0.0f; map_corner_pts_marker.color.b = 0.0f; map_corner_pts_marker.color.a = 0.9f; // RED
        map_corner_pts_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Initialization for Scan Corner Points
        scan_corner_pts_marker.header.frame_id = "camera_init";
        scan_corner_pts_marker.ns = "scan_corner_pts";
        scan_corner_pts_marker.id = 1;
        scan_corner_pts_marker.type = visualization_msgs::msg::Marker::POINTS;
        scan_corner_pts_marker.action = visualization_msgs::msg::Marker::ADD;
        scan_corner_pts_marker.pose.orientation.w = 1.0;
        scan_corner_pts_marker.scale.x = 0.05; scan_corner_pts_marker.scale.y = 0.05; scan_corner_pts_marker.scale.z = 0.05;
        scan_corner_pts_marker.color.r = 0.0f; scan_corner_pts_marker.color.g = 1.0f; scan_corner_pts_marker.color.b = 0.0f; scan_corner_pts_marker.color.a = 0.9f; // GREEN
        scan_corner_pts_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Initialization for Corner Lines
        corner_lines_marker.header.frame_id = "camera_init";
        corner_lines_marker.ns = "corner_corr_lines";
        corner_lines_marker.id = 2;
        corner_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        corner_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_lines_marker.pose.orientation.w = 1.0;
        corner_lines_marker.scale.x = 0.02; // Line width
        corner_lines_marker.color.r = 0.0f; corner_lines_marker.color.g = 0.0f; corner_lines_marker.color.b = 1.0f; corner_lines_marker.color.a = 0.8f; // BLUE
        corner_lines_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Initialization for Map Surface Points
        map_surface_pts_marker.header.frame_id = "camera_init";
        map_surface_pts_marker.ns = "map_surface_pts";
        map_surface_pts_marker.id = 3;
        map_surface_pts_marker.type = visualization_msgs::msg::Marker::POINTS;
        map_surface_pts_marker.action = visualization_msgs::msg::Marker::ADD;
        map_surface_pts_marker.pose.orientation.w = 1.0;
        map_surface_pts_marker.scale.x = 0.05; map_surface_pts_marker.scale.y = 0.05; map_surface_pts_marker.scale.z = 0.05;
        map_surface_pts_marker.color.r = 1.0f; map_surface_pts_marker.color.g = 0.0f; map_surface_pts_marker.color.b = 0.0f; map_surface_pts_marker.color.a = 0.9f; // RED
        map_surface_pts_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Initialization for Scan Surface Points
        scan_surface_pts_marker.header.frame_id = "camera_init";
        scan_surface_pts_marker.ns = "scan_surface_pts";
        scan_surface_pts_marker.id = 4;
        scan_surface_pts_marker.type = visualization_msgs::msg::Marker::POINTS;
        scan_surface_pts_marker.action = visualization_msgs::msg::Marker::ADD;
        scan_surface_pts_marker.pose.orientation.w = 1.0;
        scan_surface_pts_marker.scale.x = 0.05; scan_surface_pts_marker.scale.y = 0.05; scan_surface_pts_marker.scale.z = 0.05;
        scan_surface_pts_marker.color.r = 0.0f; scan_surface_pts_marker.color.g = 1.0f; scan_surface_pts_marker.color.b = 0.0f; scan_surface_pts_marker.color.a = 0.9f; // GREEN
        scan_surface_pts_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Initialization for Surface Lines
        surface_lines_marker.header.frame_id = "camera_init";
        surface_lines_marker.ns = "surface_corr_lines";
        surface_lines_marker.id = 5;
        surface_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        surface_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        surface_lines_marker.pose.orientation.w = 1.0;
        surface_lines_marker.scale.x = 0.02; // Line width
        surface_lines_marker.color.r = 0.0f; surface_lines_marker.color.g = 0.0f; surface_lines_marker.color.b = 1.0f; surface_lines_marker.color.a = 0.8f; // BLUE
        surface_lines_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      }
      if (markers_sel_features_) { // This is for the other marker type, ensure it's initialized if used
        selected_points_marker.header.frame_id = "camera_init";
        selected_points_marker.ns = "selected_scan_features";
        selected_points_marker.id = 2; // Different ID
        selected_points_marker.type = visualization_msgs::msg::Marker::POINTS;
        selected_points_marker.action = visualization_msgs::msg::Marker::ADD;
        selected_points_marker.pose.orientation.w = 1.0;
        selected_points_marker.scale.x = 0.06; selected_points_marker.scale.y = 0.06; selected_points_marker.scale.z = 0.06; // Point size
        selected_points_marker.color.g = 1.0f; selected_points_marker.color.a = 0.9f; // Green
        selected_points_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      }

      float deltaR_final = 0.0f; // For logging after loop
      float deltaT_final = 0.0f; // For logging after loop
      int actualIterCount = 0;

      for (int iterCount = 0; iterCount < 5; iterCount++) {
        actualIterCount = iterCount + 1;
        laserCloudOri_->clear();
        coeffSel_->clear();

        if (markers_icp_corr_) {
            map_corner_pts_marker.points.clear();
            scan_corner_pts_marker.points.clear();
            corner_lines_marker.points.clear();
            map_surface_pts_marker.points.clear();
            scan_surface_pts_marker.points.clear();
            surface_lines_marker.points.clear();
        }
        if (markers_sel_features_) {
            selected_points_marker.points.clear();
        }

        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd(5); // For nearest K search
        std::vector<float> pointSearchSqDis(5);

        // Corner feature matching and residual calculation
        for (const auto& p_orig : laserCloudCornerLast_down_->points) { // Use downsampled cloud
          pointOri = p_orig;
          pointAssociateToMap(&pointOri, &pointSel);
          kdtreeCornerFromMap_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

          if (pointSearchSqDis[4] < 1.5) { // If 5th closest point is within 1.5m
            // PCA for line fitting
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
              cx += laserCloudCornerFromMap_->points[pointSearchInd[j]].x;
              cy += laserCloudCornerFromMap_->points[pointSearchInd[j]].y;
              cz += laserCloudCornerFromMap_->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5; cz /= 5;
            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
              float ax = laserCloudCornerFromMap_->points[pointSearchInd[j]].x - cx;
              float ay = laserCloudCornerFromMap_->points[pointSearchInd[j]].y - cy;
              float az = laserCloudCornerFromMap_->points[pointSearchInd[j]].z - cz;
              a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
              a22 += ay * ay; a23 += ay * az; a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;
            matA1_.at<float>(0,0)=a11; matA1_.at<float>(0,1)=a12; matA1_.at<float>(0,2)=a13;
            matA1_.at<float>(1,0)=a12; matA1_.at<float>(1,1)=a22; matA1_.at<float>(1,2)=a23;
            matA1_.at<float>(2,0)=a13; matA1_.at<float>(2,1)=a23; matA1_.at<float>(2,2)=a33;
            cv::eigen(matA1_, matD1_, matV1_); // matD1_: eigenvalues, matV1_: eigenvectors

            if (matD1_.at<float>(0,0) > 3 * matD1_.at<float>(0,1)) { // If first eigenvalue is much larger (line feature)
              float x0 = pointSel.x; float y0 = pointSel.y; float z0 = pointSel.z;
              float x1 = cx + 0.1f * matV1_.at<float>(0,0); float y1 = cy + 0.1f * matV1_.at<float>(0,1); float z1 = cz + 0.1f * matV1_.at<float>(0,2);
              float x2 = cx - 0.1f * matV1_.at<float>(0,0); float y2 = cy - 0.1f * matV1_.at<float>(0,1); float z2 = cz - 0.1f * matV1_.at<float>(0,2);
              // Area of triangle / length of base = height (distance to line)
              float a012 = sqrt(pow((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1),2) + pow((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1),2) + pow((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1),2));
              float l12 = sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
              float la = ((y1-y2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) + (z1-z2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1))) / a012 / l12;
              float lb = -((x1-x2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) - (z1-z2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))) / a012 / l12;
              float lc = -((x1-x2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)) + (y1-y2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))) / a012 / l12;
              float ld2 = a012 / l12; // Distance from pointSel to the line
              float s_factor = 1.0f - 0.9f * std::abs(ld2);
              coeff.x = s_factor * la; coeff.y = s_factor * lb; coeff.z = s_factor * lc;
              coeff.intensity = s_factor * ld2;
              if (s_factor > 0.1 && ld2 < 0.5) { // Check distance and weighting factor
                laserCloudOri_->push_back(pointOri);
                coeffSel_->push_back(coeff);
                if (markers_icp_corr_) {
                  geometry_msgs::msg::Point p_scan;
                  geometry_msgs::msg::Point p_scan_corner;
                  p_scan_corner.x = pointSel.x; p_scan_corner.y = pointSel.y; p_scan_corner.z = pointSel.z;
                  scan_corner_pts_marker.points.push_back(p_scan_corner);

                  for(int k=0; k<5; ++k) {
                    geometry_msgs::msg::Point map_pt;
                    map_pt.x = laserCloudCornerFromMap_->points[pointSearchInd[k]].x;
                    map_pt.y = laserCloudCornerFromMap_->points[pointSearchInd[k]].y;
                    map_pt.z = laserCloudCornerFromMap_->points[pointSearchInd[k]].z;
                    map_corner_pts_marker.points.push_back(map_pt);
                  }

                  geometry_msgs::msg::Point p_map_center;
                  p_map_center.x = cx; p_map_center.y = cy; p_map_center.z = cz;
                  corner_lines_marker.points.push_back(p_scan_corner);
                  corner_lines_marker.points.push_back(p_map_center);
                }
                if (markers_sel_features_) {
                  geometry_msgs::msg::Point p_sel_geom;
                  p_sel_geom.x = pointSel.x; p_sel_geom.y = pointSel.y; p_sel_geom.z = pointSel.z;
                  selected_points_marker.points.push_back(p_sel_geom);
                }
              }
            }
          }
        }
        // Surface feature matching
        for (const auto& p_orig : laserCloudSurfLast_down_->points) {
          pointOri = p_orig;
          pointAssociateToMap(&pointOri, &pointSel);
          kdtreeSurfFromMap_->nearestKSearch(pointSel, 8, pointSearchInd, pointSearchSqDis); // 8 points for plane
          if (pointSearchSqDis[7] < 5.0) { // If 8th point is within sqrt(5)m
            for (int j=0; j<8; ++j) {
                matA0_.at<float>(j,0) = laserCloudSurfFromMap_->points[pointSearchInd[j]].x;
                matA0_.at<float>(j,1) = laserCloudSurfFromMap_->points[pointSearchInd[j]].y;
                matA0_.at<float>(j,2) = laserCloudSurfFromMap_->points[pointSearchInd[j]].z;
            }
            // matB0_ is already filled with -1.
            // Solve Ax = B for plane parameters (normal vector components / D)
            cv::solve(matA0_.rowRange(0,8), matB0_.rowRange(0,8), matX0_, cv::DECOMP_QR); // Use only 8 points
            float pa = matX0_.at<float>(0,0); float pb = matX0_.at<float>(1,0); float pc = matX0_.at<float>(2,0);
            float pd = 1.0f; // ax+by+cz+d=0, here d=1, so ax+by+cz=-1
            float ps = sqrt(pa*pa+pb*pb+pc*pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps; // Normalize plane equation

            bool planeValid = true;
            for (int j=0; j<8; ++j) { // Check if all 8 points are close to the fitted plane
              if (std::abs(pa*laserCloudSurfFromMap_->points[pointSearchInd[j]].x + 
                           pb*laserCloudSurfFromMap_->points[pointSearchInd[j]].y + 
                           pc*laserCloudSurfFromMap_->points[pointSearchInd[j]].z + pd) > 0.2) {
                planeValid = false; break;
              }
            }
            if (planeValid) {
              float dist_to_plane = pa*pointSel.x + pb*pointSel.y + pc*pointSel.z + pd;
              float s_factor = 1.0f - 0.9f * std::abs(dist_to_plane) / sqrt(sqrt(pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z)); // Weight by distance and depth
              coeff.x = s_factor * pa; coeff.y = s_factor * pb; coeff.z = s_factor * pc;
              coeff.intensity = s_factor * dist_to_plane;
              if (s_factor > 0.1 && std::abs(dist_to_plane) < 0.5) { // Check distance and weight
                laserCloudOri_->push_back(pointOri);
                coeffSel_->push_back(coeff);
                if (pub_icp_correspondence_markers_) {
                  geometry_msgs::msg::Point p_scan_surf;
                  p_scan_surf.x = pointSel.x; p_scan_surf.y = pointSel.y; p_scan_surf.z = pointSel.z;
                  geometry_msgs::msg::Point p_map_proj;
                  p_map_proj.x = pointSel.x - dist_to_plane * pa; // pa, pb, pc are normalized
                  // geometry_msgs::msg::Point p_scan_surf; // Removed redeclaration
                  p_scan_surf.x = pointSel.x; p_scan_surf.y = pointSel.y; p_scan_surf.z = pointSel.z;
                  scan_surface_pts_marker.points.push_back(p_scan_surf);

                  for(int k=0; k<8; ++k) { // The 8 points used for plane fitting
                    geometry_msgs::msg::Point map_pt;
                    map_pt.x = laserCloudSurfFromMap_->points[pointSearchInd[k]].x;
                    map_pt.y = laserCloudSurfFromMap_->points[pointSearchInd[k]].y;
                    map_pt.z = laserCloudSurfFromMap_->points[pointSearchInd[k]].z;
                    map_surface_pts_marker.points.push_back(map_pt);
                  }

                  // geometry_msgs::msg::Point p_map_proj; // Removed redeclaration
                  p_map_proj.x = pointSel.x - dist_to_plane * pa;
                  p_map_proj.y = pointSel.y - dist_to_plane * pb;
                  p_map_proj.z = pointSel.z - dist_to_plane * pc;
                  surface_lines_marker.points.push_back(p_scan_surf);
                  surface_lines_marker.points.push_back(p_map_proj);
                }
                if (markers_sel_features_) {
                  geometry_msgs::msg::Point p_sel_geom;
                  p_sel_geom.x = pointSel.x; p_sel_geom.y = pointSel.y; p_sel_geom.z = pointSel.z;
                  selected_points_marker.points.push_back(p_sel_geom);
                }
              }
            }
          }
        }
        
        // Solve for transformation increment
        int laserCloudSelNum = laserCloudOri_->points.size();
        if (enable_icp_debug_logs_) {
          RCLCPP_INFO(this->get_logger(), "ICP Iter %d: laserCloudSelNum: %d", iterCount, laserCloudSelNum);
        }

        if (enable_health_warnings_param_) {
            if (laserCloudSelNum < min_icp_correspondences_param_) {
                RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of ICP correspondences (%d) in iteration %d is below threshold (%d).", laserCloudSelNum, actualIterCount, min_icp_correspondences_param_);
            }
        }

        if (laserCloudSelNum < 50) continue; // Need enough constraints (original critical check)

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0)); // Solution (delta_transform)

        float srx = sin(transformTobeMapped_[0]); float crx = cos(transformTobeMapped_[0]);
        float sry = sin(transformTobeMapped_[1]); float cry = cos(transformTobeMapped_[1]);
        float srz = sin(transformTobeMapped_[2]); float crz = cos(transformTobeMapped_[2]);

        for (int i=0; i<laserCloudSelNum; ++i) {
          pointOri = laserCloudOri_->points[i];
          coeff = coeffSel_->points[i];
          // Jacobian calculation (derivatives of point-to-line/plane distance w.r.t transform parameters)
          // This is the core of Gauss-Newton/Levenberg-Marquardt step in ICP
          // The exact formulas depend on the chosen rotation parameterization (Euler angles here) and point-to-feature model
          // The following is a direct port of the original LOAM Jacobian for its specific Euler angle (ZXY extrinsic or YXZ intrinsic) and point transformation sequence.
          float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x +
                      (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y +
                      (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;
          float ary = ((cry*srx*srz - crz*sry)*pointOri.x + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x +
                      ((-cry*crz - srx*sry*srz)*pointOri.x + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;
          float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x +
                      (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y +
                      ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
          
          matA.at<float>(i,0) = arx; matA.at<float>(i,1) = ary; matA.at<float>(i,2) = arz;
          matA.at<float>(i,3) = coeff.x; matA.at<float>(i,4) = coeff.y; matA.at<float>(i,5) = coeff.z;
          matB.at<float>(i,0) = -coeff.intensity; // The residual (negative distance)
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) { // Deterioration judgment on first iteration
          cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0)); // Eigenvalues
          cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0)); // Eigenvectors
          cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
          cv::eigen(matAtA, matE, matV);
          matV.copyTo(matV2);
          isDegenerate_ = false;
          float eignThre[6] = {10, 10, 10, 1, 1, 1}; // Thresholds for eigenvalues (more strict for rotation)
          for (int i=5; i>=0; --i) {
            if (matE.at<float>(0,i) < eignThre[i]) {
              for (int j=0; j<6; ++j) matV2.at<float>(i,j) = 0; // Zero out eigenvector for small eigenvalue
              isDegenerate_ = true;
            } else break;
          }
          matP_ = matV.inv() * matV2; // Projection matrix for handling degeneracy
        }
        if (isDegenerate_) {
          cv::Mat matX2(6,1,CV_32F, cv::Scalar::all(0));
          matX.copyTo(matX2);
          matX = matP_ * matX2; // Project solution to non-degenerate subspace
        }

        // Update transformation
        transformTobeMapped_[0] += matX.at<float>(0,0);
        transformTobeMapped_[1] += matX.at<float>(1,0);
        transformTobeMapped_[2] += matX.at<float>(2,0);
        transformTobeMapped_[3] += matX.at<float>(3,0);
        transformTobeMapped_[4] += matX.at<float>(4,0);
        transformTobeMapped_[5] += matX.at<float>(5,0);

        // The existing deltaR and deltaT are fine for the break condition
        deltaR_final = sqrt(pow(rad2deg(matX.at<float>(0,0)),2) + pow(rad2deg(matX.at<float>(1,0)),2) + pow(rad2deg(matX.at<float>(2,0)),2));
        deltaT_final = sqrt(pow(matX.at<float>(3,0)*100,2) + pow(matX.at<float>(4,0)*100,2) + pow(matX.at<float>(5,0)*100,2));
        if (deltaR_final < 0.05 && deltaT_final < 0.05) break;
      }
      if (enable_icp_debug_logs_) {
        RCLCPP_INFO(this->get_logger(), "ICP End: Ran %d iterations. Final deltaR: %f deg, deltaT: %f cm", actualIterCount, deltaR_final, deltaT_final);
      }

      if (enable_health_warnings_param_) {
          if (deltaR_final > max_icp_delta_rotation_deg_param_) {
              RCLCPP_WARN(this->get_logger(), "LaserMapping: ICP delta rotation (%.2f deg) exceeds threshold (%.2f deg).", deltaR_final, max_icp_delta_rotation_deg_param_);
          }
          if (deltaT_final > max_icp_delta_translation_cm_param_) {
              RCLCPP_WARN(this->get_logger(), "LaserMapping: ICP delta translation (%.2f cm) exceeds threshold (%.2f cm).", deltaT_final, max_icp_delta_translation_cm_param_);
          }
          if (warn_on_icp_degeneracy_param_ && isDegenerate_) {
              RCLCPP_WARN(this->get_logger(), "LaserMapping: ICP optimization was degenerate.");
          }
      }

      if (markers_icp_corr_) {
        rclcpp::Time currentTime_for_markers = rclcpp::Time(static_cast<uint64_t>(timeLaserCloudCornerLast_ * 1e9)); // Use input cloud time for consistency

        if (map_corner_pts_marker.points.size() > 0) { map_corner_pts_marker.header.stamp = currentTime_for_markers; correspondence_marker_array.markers.push_back(map_corner_pts_marker); }
        if (scan_corner_pts_marker.points.size() > 0) { scan_corner_pts_marker.header.stamp = currentTime_for_markers; correspondence_marker_array.markers.push_back(scan_corner_pts_marker); }
        if (corner_lines_marker.points.size() > 0) { corner_lines_marker.header.stamp = currentTime_for_markers; correspondence_marker_array.markers.push_back(corner_lines_marker); }
        if (map_surface_pts_marker.points.size() > 0) { map_surface_pts_marker.header.stamp = currentTime_for_markers; correspondence_marker_array.markers.push_back(map_surface_pts_marker); }
        if (scan_surface_pts_marker.points.size() > 0) { scan_surface_pts_marker.header.stamp = currentTime_for_markers; correspondence_marker_array.markers.push_back(scan_surface_pts_marker); }
        if (surface_lines_marker.points.size() > 0) { surface_lines_marker.header.stamp = currentTime_for_markers; correspondence_marker_array.markers.push_back(surface_lines_marker); }

        if (correspondence_marker_array.markers.size() > 0) {
          pub_icp_correspondence_markers_->publish(correspondence_marker_array);
        }
      }
      if (markers_sel_features_ && selected_points_marker.points.size() > 0) { // This part remains for the other marker type
            rclcpp::Time currentTime_for_markers = rclcpp::Time(static_cast<uint64_t>(timeLaserCloudCornerLast_ * 1e9)); // Use input cloud time for consistency
            selected_points_marker.header.stamp = currentTime_for_markers; // Use same consistent time
            visualization_msgs::msg::MarkerArray selected_feature_marker_array;
            selected_feature_marker_array.markers.push_back(selected_points_marker);
            pub_selected_feature_markers_->publish(selected_feature_marker_array);
      }
      if (enable_icp_debug_logs_) {
        RCLCPP_INFO(this->get_logger(), "ICP End: transformTobeMapped_ (before update): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", transformTobeMapped_[0], transformTobeMapped_[1], transformTobeMapped_[2], transformTobeMapped_[3], transformTobeMapped_[4], transformTobeMapped_[5]);
      }
      transformUpdate(); // transformAftMapped_ = transformTobeMapped_
      if (enable_icp_debug_logs_) {
        RCLCPP_INFO(this->get_logger(), "ICP End: transformAftMapped_ (after update): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", transformAftMapped_[0], transformAftMapped_[1], transformAftMapped_[2], transformAftMapped_[3], transformAftMapped_[4], transformAftMapped_[5]);
      }
    } else {
      if (enable_icp_debug_logs_) {
        RCLCPP_INFO(this->get_logger(), "ICP SKIPPED: Not enough points in map. Corner: %ld, Surf: %ld", laserCloudCornerFromMap_->points.size(), laserCloudSurfFromMap_->points.size());
        // If ICP is skipped, still log the transforms to see if they change due to prediction alone
        RCLCPP_INFO(this->get_logger(), "ICP SKIPPED: transformTobeMapped_ (before update): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", transformTobeMapped_[0], transformTobeMapped_[1], transformTobeMapped_[2], transformTobeMapped_[3], transformTobeMapped_[4], transformTobeMapped_[5]);
      }
      transformUpdate();
      if (enable_icp_debug_logs_) {
        RCLCPP_INFO(this->get_logger(), "ICP SKIPPED: transformAftMapped_ (after update): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", transformAftMapped_[0], transformAftMapped_[1], transformAftMapped_[2], transformAftMapped_[3], transformAftMapped_[4], transformAftMapped_[5]);
      }
    }
    // t3 = clock();

    // Add current scan features to map cubes
    PointType pointSel;
    for (const auto& p_orig : *laserCloudCornerLast_) { // Use original, not downsampled
      pointAssociateToMap(&p_orig, &pointSel);
      int cubeI = static_cast<int>((pointSel.x + 25.0f) / 50.0f) + laserCloudCenWidth_;
      int cubeJ = static_cast<int>((pointSel.y + 25.0f) / 50.0f) + laserCloudCenHeight_;
      int cubeK = static_cast<int>((pointSel.z + 25.0f) / 50.0f) + laserCloudCenDepth_;
      if (pointSel.x + 25.0f < 0) cubeI--; if (pointSel.y + 25.0f < 0) cubeJ--; if (pointSel.z + 25.0f < 0) cubeK--;
      if (cubeI>=0&&cubeI<laserCloudWidth_ && cubeJ>=0&&cubeJ<laserCloudHeight_ && cubeK>=0&&cubeK<laserCloudDepth_) {
        laserCloudCornerArray_[cubeI + laserCloudWidth_*cubeJ + laserCloudWidth_*laserCloudHeight_*cubeK]->push_back(pointSel);
      }
    }
    for (const auto& p_orig : *laserCloudSurfLast_down_) { // Use downsampled surf points for map
      pointAssociateToMap(&p_orig, &pointSel);
      int cubeI = static_cast<int>((pointSel.x + 25.0f) / 50.0f) + laserCloudCenWidth_;
      int cubeJ = static_cast<int>((pointSel.y + 25.0f) / 50.0f) + laserCloudCenHeight_;
      int cubeK = static_cast<int>((pointSel.z + 25.0f) / 50.0f) + laserCloudCenDepth_;
      if (pointSel.x + 25.0f < 0) cubeI--; if (pointSel.y + 25.0f < 0) cubeJ--; if (pointSel.z + 25.0f < 0) cubeK--;
      if (cubeI>=0&&cubeI<laserCloudWidth_ && cubeJ>=0&&cubeJ<laserCloudHeight_ && cubeK>=0&&cubeK<laserCloudDepth_) {
        laserCloudSurfArray_[cubeI + laserCloudWidth_*cubeJ + laserCloudWidth_*laserCloudHeight_*cubeK]->push_back(pointSel);
      }
    }
    // Downsample map cubes that were updated
    for (int i=0; i<laserCloudValidNum_; ++i) {
      int ind = laserCloudValidInd_[i];
      laserCloudCornerArray2_[ind]->clear();
      downSizeFilterCorner_.setInputCloud(laserCloudCornerArray_[ind]);
      downSizeFilterCorner_.filter(*laserCloudCornerArray2_[ind]);
      // Swap pointers to avoid copy
      laserCloudCornerArray_[ind].swap(laserCloudCornerArray2_[ind]);

      laserCloudSurfArray2_[ind]->clear();
      downSizeFilterSurf_.setInputCloud(laserCloudSurfArray_[ind]);
      downSizeFilterSurf_.filter(*laserCloudSurfArray2_[ind]);
      laserCloudSurfArray_[ind].swap(laserCloudSurfArray2_[ind]);
    }
    
    // Publish map surround and full resolution cloud
    laserCloudSurround2_->clear();
    laserCloudSurround2_corner_->clear();
    for (int i=0; i<laserCloudSurroundNum_; ++i) {
      int ind = laserCloudSurroundInd_[i];
      *laserCloudSurround2_corner_ += *laserCloudCornerArray_[ind];
      *laserCloudSurround2_ += *laserCloudSurfArray_[ind];
    }
    sensor_msgs::msg::PointCloud2 tempCloudMsg;
    rclcpp::Time currentTime = rclcpp::Time(static_cast<uint64_t>(timeLaserCloudCornerLast_ * 1e9));

    pcl::toROSMsg(*laserCloudSurround2_, tempCloudMsg);
    tempCloudMsg.header.stamp = currentTime;
    tempCloudMsg.header.frame_id = "camera_init";
    pubLaserCloudSurround_->publish(tempCloudMsg);

    pcl::toROSMsg(*laserCloudSurround2_corner_, tempCloudMsg);
    tempCloudMsg.header.stamp = currentTime;
    tempCloudMsg.header.frame_id = "camera_init";
    pubLaserCloudSurroundCorner_->publish(tempCloudMsg);

    laserCloudFullResColor_->clear(); // Clear before filling
    *laserCloudFullRes2_ = *laserCloudFullRes_; // Copy full res scan
    for (const auto& pt : laserCloudFullRes2_->points) {
      pcl::PointXYZRGB temp_rgb_pt;
      // The original code used pointAssociateToMap_all for full cloud, which interpolates pose
      // However, RGBpointAssociateToMap was also present. For final map coloring, transformAftMapped is fine.
      // If deskewing of the full cloud for visualization is desired, pointAssociateToMap_all should be used.
      // Let's use RGBpointAssociateToMap which uses transformAftMapped for simplicity here.
      RGBpointAssociateToMap(&pt, &temp_rgb_pt); 
      laserCloudFullResColor_->push_back(temp_rgb_pt);
    }
    pcl::toROSMsg(*laserCloudFullResColor_, tempCloudMsg);
    tempCloudMsg.header.stamp = currentTime;
    tempCloudMsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes_->publish(tempCloudMsg);
    *laserCloudFullResColor_pcd_ += *laserCloudFullResColor_; // Accumulate for saving

    // Publish odometry and TF
    // Original RPY to Quaternion: tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);
    // This means: yaw = transformAftMapped[2], pitch = -transformAftMapped[0], roll = -transformAftMapped[1]
    tf2::Quaternion q_aft_mapped;
    q_aft_mapped.setRPY(-transformAftMapped_[1], -transformAftMapped_[0], transformAftMapped_[2]); // roll, pitch, yaw
    
    odomAftMapped_.header.stamp = currentTime;
    // The original code had a specific remapping of quaternion components:
    // odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
    // odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
    // odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    // odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    // This is unusual and might be due to a coordinate system convention or a specific TF tree structure.
    // For a standard ROS right-handed system (e.g., FLU for robot, ENU for map), direct use of tf2::toMsg(q_aft_mapped) is typical.
    // I will use direct conversion first. If orientation is wrong, this part needs review based on frame definitions.
    odomAftMapped_.pose.pose.orientation = tf2::toMsg(q_aft_mapped);
    odomAftMapped_.pose.pose.position.x = transformAftMapped_[3];
    odomAftMapped_.pose.pose.position.y = transformAftMapped_[4];
    odomAftMapped_.pose.pose.position.z = transformAftMapped_[5];
    pubOdomAftMapped_->publish(odomAftMapped_);

    geometry_msgs::msg::TransformStamped tfStamped;
    tfStamped.header.stamp = this->get_clock()->now();
    tfStamped.header.frame_id = "camera_init"; // map frame
    tfStamped.child_frame_id = "aft_mapped";  // robot frame
    tfStamped.transform.translation.x = transformAftMapped_[3];
    tfStamped.transform.translation.y = transformAftMapped_[4];
    tfStamped.transform.translation.z = transformAftMapped_[5];
    tfStamped.transform.rotation = tf2::toMsg(q_aft_mapped);
    tf_broadcaster_->sendTransform(tfStamped);

    kfNum_++;
    if (kfNum_ >= 20) { // Save keyframe pose every 20 frames
      Eigen::Matrix<float,7,1> kf_pose_eigen; // x,y,z, qx,qy,qz,qw
      kf_pose_eigen << tf2::toMsg(q_aft_mapped).x, tf2::toMsg(q_aft_mapped).y, tf2::toMsg(q_aft_mapped).z, tf2::toMsg(q_aft_mapped).w,
                       transformAftMapped_[3], transformAftMapped_[4], transformAftMapped_[5];
      keyframe_pose_.push_back(kf_pose_eigen);
      kfNum_ = 0;
    }
    // t4 = clock();
    // RCLCPP_INFO(this->get_logger(), "Mapping time: Preproc=%fms, ICP=%fms, Postproc=%fms", 
    //             (double)(t2-t1)/CLOCKS_PER_SEC*1000, (double)(t3-t2)/CLOCKS_PER_SEC*1000, (double)(t4-t3)/CLOCKS_PER_SEC*1000);
  }

}; // LaserMapping Class End


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto laser_mapping_node = std::make_shared<LaserMapping>();
  
  // Use a MultiThreadedExecutor if callbacks could run in parallel or if timers need dedicated threads.
  // For this LOAM-style node where data dependencies are tight, SingleThreadedExecutor is usually fine.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(laser_mapping_node);
  executor.spin();
  // rclcpp::spin(laser_mapping_node); // Simpler alternative if no custom executor needed

  // Map saving is now handled in the destructor or a dedicated shutdown function
  rclcpp::shutdown();
  return 0;
}
