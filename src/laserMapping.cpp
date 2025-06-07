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

#include "adaptive_parameter_manager_types.h" // For LaserMappingHealth enum
#include "std_msgs/msg/int32.hpp"           // To publish enum as integer

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

    // Parameters that can be dynamically adjusted
    this->declare_parameter<double>("filter_parameter_corner", 0.2,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Voxel grid leaf size for corner points in laserMapping.")
            .set__read_only(false)); // Explicitly mark as not read-only for clarity
    this->get_parameter("filter_parameter_corner", filter_param_corner_);
    RCLCPP_INFO(this->get_logger(), "Initial filter parameter corner: %f", filter_param_corner_);
    
    this->declare_parameter<double>("filter_parameter_surf", 0.4,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Voxel grid leaf size for surface points in laserMapping.")
            .set__read_only(false));
    this->get_parameter("filter_parameter_surf", filter_param_surf_);
    RCLCPP_INFO(this->get_logger(), "Initial filter parameter surf: %f", filter_param_surf_);


    RCLCPP_INFO(this->get_logger(), "--- LaserMapping Node Parameters ---");
    RCLCPP_INFO(this->get_logger(), "map_file_path: %s", map_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "filter_parameter_corner: %f", filter_param_corner_);
    RCLCPP_INFO(this->get_logger(), "filter_parameter_surf: %f", filter_param_surf_);

    // Declare and get health monitoring parameters
    this->declare_parameter<bool>("health.enable_health_warnings", true);
    this->declare_parameter<int>("health.min_downsampled_corner_features", 12);
    this->declare_parameter<int>("health.min_downsampled_surf_features", 30);
    this->declare_parameter<int>("health.min_map_corner_points_for_icp", 30);
    this->declare_parameter<int>("health.min_map_surf_points_for_icp", 100);
    this->declare_parameter<int>("health.min_icp_correspondences", 40);
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
    // ... (Log messages for health params remain the same)
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
    laserCloudFullRes2_ = std::make_shared<pcl::PointCloud<PointType>>();
    laserCloudFullResColor_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    laserCloudFullResColor_pcd_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    kdtreeCornerFromMap_ = std::make_shared<pcl::KdTreeFLANN<PointType>>();
    kdtreeSurfFromMap_ = std::make_shared<pcl::KdTreeFLANN<PointType>>();
    
    laserCloudCornerArray_.resize(laserCloudNum_);
    laserCloudSurfArray_.resize(laserCloudNum_);
    laserCloudCornerArray2_.resize(laserCloudNum_);
    laserCloudSurfArray2_.resize(laserCloudNum_);
    for (int i = 0; i < laserCloudNum_; ++i) {
      laserCloudCornerArray_[i] = std::make_shared<pcl::PointCloud<PointType>>();
      laserCloudSurfArray_[i] = std::make_shared<pcl::PointCloud<PointType>>();
      laserCloudCornerArray2_[i] = std::make_shared<pcl::PointCloud<PointType>>();
      laserCloudSurfArray2_[i] = std::make_shared<pcl::PointCloud<PointType>>();
    }
    
    transformTobeMapped_.fill(0.0f);
    transformAftMapped_.fill(0.0f);
    transformLastMapped_.fill(0.0f);

    matA0_ = cv::Mat(10, 3, CV_32F, cv::Scalar::all(0));
    matB0_ = cv::Mat(10, 1, CV_32F, cv::Scalar::all(-1));
    matX0_ = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0)); // Corrected size to 3x1
    matA1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1_ = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    matV1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matP_ = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    downSizeFilterCorner_.setLeafSize(filter_param_corner_, filter_param_corner_, filter_param_corner_);
    downSizeFilterSurf_.setLeafSize(filter_param_surf_, filter_param_surf_, filter_param_surf_);

    pubLaserCloudSurround_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround", 100);
    pubLaserCloudSurroundCorner_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround_corner", 100);
    pubLaserCloudFullRes_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered", 100);
    pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100);
    pub_icp_correspondence_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/icp_correspondences", 10);
    pub_selected_feature_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/selected_features", 10);
    pub_health_status_ = this->create_publisher<std_msgs::msg::Int32>("/laser_mapping/health_status", 10);

    odomAftMapped_.header.frame_id = "camera_init";
    odomAftMapped_.child_frame_id = "aft_mapped";

    subLaserCloudCornerLast_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laser_cloud_sharp", rclcpp::QoS(100), std::bind(&LaserMapping::laserCloudCornerLastHandler, this, std::placeholders::_1));
    subLaserCloudSurfLast_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laser_cloud_flat", rclcpp::QoS(100), std::bind(&LaserMapping::laserCloudSurfLastHandler, this, std::placeholders::_1));
    subLaserCloudFullRes_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox_cloud", rclcpp::QoS(100), std::bind(&LaserMapping::laserCloudFullResHandler, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "--- LaserMapping Subscribed Topics ---");
    // ... (Log messages for subscriptions remain the same)
    RCLCPP_INFO(this->get_logger(), "------------------------------------");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    processing_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&LaserMapping::processMappingLoop, this));

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LaserMapping::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LaserMapping Node Initialized.");
  }

  ~LaserMapping() {
    RCLCPP_INFO(this->get_logger(), "LaserMapping Node Shutting down. Saving map...");
    saveMap();
  }


private:
  // Parameter callback handle
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Member Variables
  double timeLaserCloudCornerLast_, timeLaserCloudSurfLast_, timeLaserCloudFullRes_;
  bool newLaserCloudCornerLast_ = false, newLaserCloudSurfLast_ = false, newLaserCloudFullRes_ = false;

  int laserCloudCenWidth_, laserCloudCenHeight_, laserCloudCenDepth_;
  const int laserCloudWidth_, laserCloudHeight_, laserCloudDepth_;
  const int laserCloudNum_;

  std::vector<int> laserCloudValidInd_;
  std::vector<int> laserCloudSurroundInd_;
  int laserCloudValidNum_ = 0;
  int laserCloudSurroundNum_ = 0;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_, laserCloudCornerLast_down_;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_, laserCloudSurfLast_down_;
  pcl::PointCloud<PointType>::Ptr laserCloudOri_, coeffSel_;
  pcl::PointCloud<PointType>::Ptr laserCloudSurround2_, laserCloudSurround2_corner_;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_, laserCloudSurfFromMap_;
  pcl::PointCloud<PointType>::Ptr laserCloudFullRes_, laserCloudFullRes2_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_, laserCloudFullResColor_pcd_;

  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray_;
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray_;
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray2_;
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray2_;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap_, kdtreeSurfFromMap_;

  std::array<float, 6> transformTobeMapped_;
  std::array<float, 6> transformAftMapped_;
  std::array<float, 6> transformLastMapped_;
  
  std::vector<Eigen::Matrix<float,7,1>> keyframe_pose_;
  int kfNum_;

  cv::Mat matA0_, matB0_, matX0_;
  cv::Mat matA1_, matD1_, matV1_;
  cv::Mat matP_;

  bool isDegenerate_ = false;
  // Store last ICP iteration results for health reporting (simplified)
  float last_icp_delta_rotation_ = 0.0f;
  float last_icp_delta_translation_ = 0.0f;
  int last_icp_correspondences_ = 0;


  pcl::VoxelGrid<PointType> downSizeFilterCorner_;
  pcl::VoxelGrid<PointType> downSizeFilterSurf_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurroundCorner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_icp_correspondence_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_selected_feature_markers_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_health_status_;
  
  nav_msgs::msg::Odometry odomAftMapped_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudCornerLast_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudSurfLast_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr processing_timer_;

  std::string map_file_path_;
  double filter_param_corner_;
  double filter_param_surf_;
  bool markers_icp_corr_;
  bool markers_sel_features_;
  bool enable_icp_debug_logs_;

  bool enable_health_warnings_param_;
  int min_downsampled_corner_features_param_;
  int min_downsampled_surf_features_param_;
  int min_map_corner_points_for_icp_param_;
  int min_map_surf_points_for_icp_param_;
  int min_icp_correspondences_param_;
  double max_icp_delta_rotation_deg_param_;
  double max_icp_delta_translation_cm_param_;
  bool warn_on_icp_degeneracy_param_;

  Eigen::Matrix4f trans_euler_to_matrix(const std::array<float, 6>& trans) {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      Eigen::Matrix3f R;
      Eigen::AngleAxisf rollAngle(trans[0], Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf pitchAngle(trans[1], Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf yawAngle(trans[2], Eigen::Vector3f::UnitZ());
      R = pitchAngle * rollAngle * yawAngle;
      T.block<3,3>(0,0) = R;
      T.block<3,1>(0,3) = Eigen::Vector3f(trans[3], trans[4], trans[5]);
      return T;
  }

  void transformAssociateToMap() {
    Eigen::Matrix4f T_aft = trans_euler_to_matrix(transformAftMapped_);
    Eigen::Matrix4f T_last = trans_euler_to_matrix(transformLastMapped_);
    Eigen::Matrix4f T_predict = T_aft * (T_last.inverse() * T_aft);
    Eigen::Matrix3f R_predict = T_predict.block<3,3>(0,0);
    Eigen::Vector3f euler_predict_angles = R_predict.eulerAngles(1,0,2);
    transformTobeMapped_[0] = euler_predict_angles[1];
    transformTobeMapped_[1] = euler_predict_angles[0];
    transformTobeMapped_[2] = euler_predict_angles[2];
    transformTobeMapped_[3] = T_predict.coeff(0,3);
    transformTobeMapped_[4] = T_predict.coeff(1,3);
    transformTobeMapped_[5] = T_predict.coeff(2,3);
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

    float x1 = cos(rz) * pi->x - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x + cos(rz) * pi->y;
    float z1 = pi->z;
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;
    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    po->intensity = pi->intensity;
  }
  
  void pointAssociateToMap_all(PointType const * const pi, PointType * const po) {
    float s = 0.0f;
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
    po->intensity = pi->intensity;
  }

  void RGBpointAssociateToMap(PointType const * const pi, pcl::PointXYZRGB * const po) {
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
    float intensity_val = pi->intensity;
    float reflectivity = (intensity_val - std::floor(intensity_val)) * 10000.0f;
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

  void laserCloudCornerLastHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->data.empty()) { return; }
    timeLaserCloudCornerLast_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    laserCloudCornerLast_->clear();
    pcl::fromROSMsg(*msg, *laserCloudCornerLast_);
    newLaserCloudCornerLast_ = true;
  }

  void laserCloudSurfLastHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->data.empty()) { return; }
    timeLaserCloudSurfLast_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    laserCloudSurfLast_->clear();
    pcl::fromROSMsg(*msg, *laserCloudSurfLast_);
    newLaserCloudSurfLast_ = true;
  }

  void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->data.empty()) { return; }
    timeLaserCloudFullRes_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    laserCloudFullRes_->clear();
    pcl::fromROSMsg(*msg, *laserCloudFullRes_);
    newLaserCloudFullRes_ = true;
  }
  
  void saveMap() {
    if (!rclcpp::ok()) {
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
    } else { RCLCPP_ERROR(this->get_logger(), "Failed to open keyframe_file for writing: %s", (map_file_path_ + "/key_frame.txt").c_str()); }
    
    pcl::PointCloud<PointType>::Ptr mapSurfTotal(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr mapCornerTotal(new pcl::PointCloud<PointType>());
    for(int i=0; i<laserCloudNum_; ++i) {
        *mapSurfTotal += *laserCloudSurfArray_[i];
        *mapCornerTotal += *laserCloudCornerArray_[i];
    }
    pcl::PCDWriter pcd_writer;
    if (mapSurfTotal->size() > 0) pcd_writer.writeBinary(surf_filename, *mapSurfTotal);
    if (mapCornerTotal->size() > 0) pcd_writer.writeBinary(corner_filename, *mapCornerTotal);
    if (laserCloudFullResColor_pcd_->size() > 0) pcd_writer.writeBinary(all_points_filename, *laserCloudFullResColor_pcd_);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            std::string param_name = param.get_name();
            RCLCPP_INFO(this->get_logger(), "Received parameter update for: %s of type %s", param_name.c_str(), param.get_type_name().c_str());

            if (param_name == "filter_parameter_corner") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    filter_param_corner_ = param.as_double();
                    downSizeFilterCorner_.setLeafSize(filter_param_corner_, filter_param_corner_, filter_param_corner_);
                    RCLCPP_INFO(this->get_logger(), "Updated corner filter leaf size to: %f", filter_param_corner_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid type for parameter filter_parameter_corner. Expected double.");
                    result.successful = false;
                }
            } else if (param_name == "filter_parameter_surf") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    filter_param_surf_ = param.as_double();
                    downSizeFilterSurf_.setLeafSize(filter_param_surf_, filter_param_surf_, filter_param_surf_);
                    RCLCPP_INFO(this->get_logger(), "Updated surface filter leaf size to: %f", filter_param_surf_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid type for parameter filter_parameter_surf. Expected double.");
                    result.successful = false;
                }
            }
        }
        return result;
    }

  void processMappingLoop() {
    if (!(newLaserCloudCornerLast_ && newLaserCloudSurfLast_ && newLaserCloudFullRes_ &&
        std::abs(timeLaserCloudSurfLast_ - timeLaserCloudCornerLast_) < 0.005 &&
        std::abs(timeLaserCloudFullRes_ - timeLaserCloudCornerLast_) < 0.005)) {
      return;
    }

    newLaserCloudCornerLast_ = false;
    newLaserCloudSurfLast_ = false;
    newLaserCloudFullRes_ = false;

    transformAssociateToMap();

    PointType pointOnYAxis;
    pointOnYAxis.x = 0.0; pointOnYAxis.y = 10.0; pointOnYAxis.z = 0.0;
    pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

    int centerCubeI = static_cast<int>((transformTobeMapped_[3] + 25.0) / 50.0) + laserCloudCenWidth_;
    int centerCubeJ = static_cast<int>((transformTobeMapped_[4] + 25.0) / 50.0) + laserCloudCenHeight_;
    int centerCubeK = static_cast<int>((transformTobeMapped_[5] + 25.0) / 50.0) + laserCloudCenDepth_;
    if (transformTobeMapped_[3] + 25.0 < 0) centerCubeI--;
    if (transformTobeMapped_[4] + 25.0 < 0) centerCubeJ--;
    if (transformTobeMapped_[5] + 25.0 < 0) centerCubeK--;

    while (centerCubeI < 3) { /* ... map shifting logic ... */
        for (int j = 0; j < laserCloudHeight_; j++) { for (int k = 0; k < laserCloudDepth_; k++) {
        pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
        pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
        for (int i = laserCloudWidth_ - 1; i >= 1; i--) {
            laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudCornerArray_[i - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
            laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudSurfArray_[i - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
        }
        laserCloudCornerArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
        laserCloudSurfArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
        tempCorner->clear(); tempSurf->clear(); }} centerCubeI++; laserCloudCenWidth_++;
    }
    while (centerCubeI >= laserCloudWidth_ - 3) { /* ... */
        for (int j = 0; j < laserCloudHeight_; j++) { for (int k = 0; k < laserCloudDepth_; k++) {
        pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
        pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[0 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
        for (int i = 0; i < laserCloudWidth_ - 1; i++) {
            laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudCornerArray_[i + 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
            laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudSurfArray_[i + 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k];
        }
        laserCloudCornerArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
        laserCloudSurfArray_[laserCloudWidth_ - 1 + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
        tempCorner->clear(); tempSurf->clear(); }} centerCubeI--; laserCloudCenWidth_--;
    }
    while (centerCubeJ < 3) { /* ... */
        for (int i = 0; i < laserCloudWidth_; i++) { for (int k = 0; k < laserCloudDepth_; k++) {
        pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * (laserCloudHeight_ - 1) + laserCloudWidth_ * laserCloudHeight_ * k];
        pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * (laserCloudHeight_ - 1) + laserCloudWidth_ * laserCloudHeight_ * k];
        for (int j = laserCloudHeight_ - 1; j >= 1; j--) {
            laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudCornerArray_[i + laserCloudWidth_ * (j-1) + laserCloudWidth_ * laserCloudHeight_ * k];
            laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudSurfArray_[i + laserCloudWidth_ * (j-1) + laserCloudWidth_ * laserCloudHeight_ * k];
        }
        laserCloudCornerArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
        laserCloudSurfArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
        tempCorner->clear(); tempSurf->clear(); }} centerCubeJ++; laserCloudCenHeight_++;
    }
    while (centerCubeJ >= laserCloudHeight_ - 3) { /* ... */
        for (int i = 0; i < laserCloudWidth_; i++) { for (int k = 0; k < laserCloudDepth_; k++) {
        pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k];
        pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * 0 + laserCloudWidth_ * laserCloudHeight_ * k];
        for (int j = 0; j < laserCloudHeight_ - 1; j++) {
            laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudCornerArray_[i + laserCloudWidth_ * (j+1) + laserCloudWidth_ * laserCloudHeight_ * k];
            laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudSurfArray_[i + laserCloudWidth_ * (j+1) + laserCloudWidth_ * laserCloudHeight_ * k];
        }
        laserCloudCornerArray_[i + laserCloudWidth_ * (laserCloudHeight_-1) + laserCloudWidth_ * laserCloudHeight_ * k] = tempCorner;
        laserCloudSurfArray_[i + laserCloudWidth_ * (laserCloudHeight_-1) + laserCloudWidth_ * laserCloudHeight_ * k] = tempSurf;
        tempCorner->clear(); tempSurf->clear(); }} centerCubeJ--; laserCloudCenHeight_--;
    }
    while (centerCubeK < 3) { /* ... */
        for (int i = 0; i < laserCloudWidth_; i++) { for (int j = 0; j < laserCloudHeight_; j++) {
        pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_ - 1)];
        pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_ - 1)];
        for (int k = laserCloudDepth_ - 1; k >= 1; k--) {
            laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k-1)];
            laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k-1)];
        }
        laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0] = tempCorner;
        laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0] = tempSurf;
        tempCorner->clear(); tempSurf->clear(); }} centerCubeK++; laserCloudCenDepth_++;
    }
    while (centerCubeK >= laserCloudDepth_ - 3) { /* ... */
        for (int i = 0; i < laserCloudWidth_; i++) { for (int j = 0; j < laserCloudHeight_; j++) {
        pcl::PointCloud<PointType>::Ptr tempCorner = laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0];
        pcl::PointCloud<PointType>::Ptr tempSurf = laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * 0];
        for (int k = 0; k < laserCloudDepth_ - 1; k++) {
            laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k+1)];
            laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k] = laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (k+1)];
        }
        laserCloudCornerArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_-1)] = tempCorner;
        laserCloudSurfArray_[i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * (laserCloudDepth_-1)] = tempSurf;
        tempCorner->clear(); tempSurf->clear(); }} centerCubeK--; laserCloudCenDepth_--;
    }

    laserCloudValidNum_ = 0; laserCloudSurroundNum_ = 0;
    laserCloudValidInd_.assign(125, 0); laserCloudSurroundInd_.assign(125, 0);
    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) { for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) { for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
    if (i >= 0 && i < laserCloudWidth_ && j >= 0 && j < laserCloudHeight_ && k >= 0 && k < laserCloudDepth_) {
        float centerX = 50.0f * (i - laserCloudCenWidth_); float centerY = 50.0f * (j - laserCloudCenHeight_); float centerZ = 50.0f * (k - laserCloudCenDepth_);
        bool isInLaserFOV = false;
        for (int ii = -1; ii <= 1; ii += 2) { for (int jj = -1; jj <= 1; jj += 2) { for (int kk = -1; kk <= 1; kk += 2) {
            float cornerX = centerX + 25.0f * ii; float cornerY = centerY + 25.0f * jj; float cornerZ = centerZ + 25.0f * kk;
            float squaredSide1 = (transformTobeMapped_[3] - cornerX) * (transformTobeMapped_[3] - cornerX) + (transformTobeMapped_[4] - cornerY) * (transformTobeMapped_[4] - cornerY) + (transformTobeMapped_[5] - cornerZ) * (transformTobeMapped_[5] - cornerZ);
            float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY) + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);
            float check1 = 100.0f + squaredSide1 - squaredSide2 - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);
            float check2 = 100.0f + squaredSide1 - squaredSide2 + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);
            if (check1 < 0 && check2 > 0) { isInLaserFOV = true; break; }
        } if(isInLaserFOV) break; } if(isInLaserFOV) break; }
        int cubeIdx = i + laserCloudWidth_ * j + laserCloudWidth_ * laserCloudHeight_ * k;
        if (isInLaserFOV && laserCloudValidNum_ < 125) laserCloudValidInd_[laserCloudValidNum_++] = cubeIdx;
        if (laserCloudSurroundNum_ < 125) laserCloudSurroundInd_[laserCloudSurroundNum_++] = cubeIdx;
    }}}}
    laserCloudCornerFromMap_->clear(); laserCloudSurfFromMap_->clear();
    for (int i = 0; i < laserCloudValidNum_; i++) {
      *laserCloudCornerFromMap_ += *laserCloudCornerArray_[laserCloudValidInd_[i]];
      *laserCloudSurfFromMap_ += *laserCloudSurfArray_[laserCloudValidInd_[i]];
    }
    laserCloudCornerLast_down_->clear();
    downSizeFilterCorner_.setInputCloud(laserCloudCornerLast_);
    downSizeFilterCorner_.filter(*laserCloudCornerLast_down_);
    laserCloudSurfLast_down_->clear();
    downSizeFilterSurf_.setInputCloud(laserCloudSurfLast_);
    downSizeFilterSurf_.filter(*laserCloudSurfLast_down_);

    if (enable_health_warnings_param_) {
        if (static_cast<int>(laserCloudCornerLast_down_->size()) < min_downsampled_corner_features_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of downsampled corner features (%ld) is below threshold (%d).", laserCloudCornerLast_down_->size(), min_downsampled_corner_features_param_);
        if (static_cast<int>(laserCloudSurfLast_down_->size()) < min_downsampled_surf_features_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of downsampled surf features (%ld) is below threshold (%d).", laserCloudSurfLast_down_->size(), min_downsampled_surf_features_param_);
    }
    if (enable_health_warnings_param_) {
        if (static_cast<int>(laserCloudCornerFromMap_->points.size()) < min_map_corner_points_for_icp_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of map corner points for ICP (%ld) is below threshold (%d).", laserCloudCornerFromMap_->points.size(), min_map_corner_points_for_icp_param_);
        if (static_cast<int>(laserCloudSurfFromMap_->points.size()) < min_map_surf_points_for_icp_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of map surf points for ICP (%ld) is below threshold (%d).", laserCloudSurfFromMap_->points.size(), min_map_surf_points_for_icp_param_);
    }

    if (laserCloudCornerFromMap_->points.size() > 10 && laserCloudSurfFromMap_->points.size() > 100) {
      kdtreeCornerFromMap_->setInputCloud(laserCloudCornerFromMap_);
      kdtreeSurfFromMap_->setInputCloud(laserCloudSurfFromMap_);
      visualization_msgs::msg::MarkerArray correspondence_marker_array;
      visualization_msgs::msg::Marker map_corner_pts_marker, scan_corner_pts_marker, corner_lines_marker;
      visualization_msgs::msg::Marker map_surface_pts_marker, scan_surface_pts_marker, surface_lines_marker;
      visualization_msgs::msg::Marker selected_points_marker;
      if (markers_icp_corr_) { /* ... marker init ... */ }
      if (markers_sel_features_) { /* ... marker init ... */ }

      float deltaR_final = 0.0f; float deltaT_final = 0.0f; int actualIterCount = 0;
      last_icp_correspondences_ = 0; // Reset for this iteration

      for (int iterCount = 0; iterCount < 5; iterCount++) { // Max 5 iterations in this version
        actualIterCount = iterCount + 1;
        laserCloudOri_->clear(); coeffSel_->clear();
        if (markers_icp_corr_) { /* ... clear marker points ... */ }
        if (markers_sel_features_) { /* ... clear marker points ... */ }
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd(8); std::vector<float> pointSearchSqDis(8); // Increased size to 8 for surf

        for (const auto& p_orig : laserCloudCornerLast_down_->points) { /* ... corner matching ... */
            pointOri = p_orig; pointAssociateToMap(&pointOri, &pointSel); kdtreeCornerFromMap_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[4] < 1.5) {
                float cx = 0, cy = 0, cz = 0; for (int j = 0; j < 5; j++) { cx += laserCloudCornerFromMap_->points[pointSearchInd[j]].x; cy += laserCloudCornerFromMap_->points[pointSearchInd[j]].y; cz += laserCloudCornerFromMap_->points[pointSearchInd[j]].z; }
                cx /= 5; cy /= 5; cz /= 5; float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) { float ax = laserCloudCornerFromMap_->points[pointSearchInd[j]].x - cx; float ay = laserCloudCornerFromMap_->points[pointSearchInd[j]].y - cy; float az = laserCloudCornerFromMap_->points[pointSearchInd[j]].z - cz; a11 += ax * ax; a12 += ax * ay; a13 += ax * az; a22 += ay * ay; a23 += ay * az; a33 += az * az; }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5; matA1_.at<float>(0,0)=a11; matA1_.at<float>(0,1)=a12; matA1_.at<float>(0,2)=a13; matA1_.at<float>(1,0)=a12; matA1_.at<float>(1,1)=a22; matA1_.at<float>(1,2)=a23; matA1_.at<float>(2,0)=a13; matA1_.at<float>(2,1)=a23; matA1_.at<float>(2,2)=a33;
                cv::eigen(matA1_, matD1_, matV1_);
                if (matD1_.at<float>(0,0) > 3 * matD1_.at<float>(0,1)) {
                    float x0 = pointSel.x; float y0 = pointSel.y; float z0 = pointSel.z; float x1 = cx + 0.1f * matV1_.at<float>(0,0); float y1 = cy + 0.1f * matV1_.at<float>(0,1); float z1 = cz + 0.1f * matV1_.at<float>(0,2); float x2 = cx - 0.1f * matV1_.at<float>(0,0); float y2 = cy - 0.1f * matV1_.at<float>(0,1); float z2 = cz - 0.1f * matV1_.at<float>(0,2);
                    float a012 = sqrt(pow((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1),2) + pow((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1),2) + pow((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1),2)); float l12 = sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
                    float la = ((y1-y2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) + (z1-z2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1))) / a012 / l12; float lb = -((x1-x2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) - (z1-z2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))) / a012 / l12; float lc = -((x1-x2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)) + (y1-y2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))) / a012 / l12;
                    float ld2 = a012 / l12; float s_factor = 1.0f - 0.9f * std::abs(ld2); coeff.x = s_factor * la; coeff.y = s_factor * lb; coeff.z = s_factor * lc; coeff.intensity = s_factor * ld2;
                    if (s_factor > 0.1 && ld2 < 0.5) { laserCloudOri_->push_back(pointOri); coeffSel_->push_back(coeff); /* ... marker logic ... */ }
                }
            }
        }
        for (const auto& p_orig : laserCloudSurfLast_down_->points) { /* ... surface matching ... */
            pointOri = p_orig; pointAssociateToMap(&pointOri, &pointSel); kdtreeSurfFromMap_->nearestKSearch(pointSel, 8, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[7] < 5.0) {
                for (int j=0; j<8; ++j) { matA0_.at<float>(j,0) = laserCloudSurfFromMap_->points[pointSearchInd[j]].x; matA0_.at<float>(j,1) = laserCloudSurfFromMap_->points[pointSearchInd[j]].y; matA0_.at<float>(j,2) = laserCloudSurfFromMap_->points[pointSearchInd[j]].z; }
                cv::solve(matA0_.rowRange(0,8), matB0_.rowRange(0,8), matX0_, cv::DECOMP_QR);
                float pa = matX0_.at<float>(0,0); float pb = matX0_.at<float>(1,0); float pc = matX0_.at<float>(2,0); float pd = 1.0f; float ps = sqrt(pa*pa+pb*pb+pc*pc); pa /= ps; pb /= ps; pc /= ps; pd /= ps;
                bool planeValid = true; for (int j=0; j<8; ++j) { if (std::abs(pa*laserCloudSurfFromMap_->points[pointSearchInd[j]].x + pb*laserCloudSurfFromMap_->points[pointSearchInd[j]].y + pc*laserCloudSurfFromMap_->points[pointSearchInd[j]].z + pd) > 0.2) { planeValid = false; break; }}
                if (planeValid) {
                    float dist_to_plane = pa*pointSel.x + pb*pointSel.y + pc*pointSel.z + pd; float s_factor = 1.0f - 0.9f * std::abs(dist_to_plane) / sqrt(sqrt(pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z));
                    coeff.x = s_factor * pa; coeff.y = s_factor * pb; coeff.z = s_factor * pc; coeff.intensity = s_factor * dist_to_plane;
                    if (s_factor > 0.1 && std::abs(dist_to_plane) < 0.5) { laserCloudOri_->push_back(pointOri); coeffSel_->push_back(coeff); /* ... marker logic ... */ }
                }
            }
        }
        int laserCloudSelNum = laserCloudOri_->points.size();
        last_icp_correspondences_ = laserCloudSelNum; // Store for health reporting
        if (enable_health_warnings_param_ && laserCloudSelNum < min_icp_correspondences_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: Number of ICP correspondences (%d) in iteration %d is below threshold (%d).", laserCloudSelNum, actualIterCount, min_icp_correspondences_param_);
        if (laserCloudSelNum < 50) continue;
        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0)); cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0)); cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0)); cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0)); cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0)); cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        float srx = sin(transformTobeMapped_[0]); float crx = cos(transformTobeMapped_[0]); float sry = sin(transformTobeMapped_[1]); float cry = cos(transformTobeMapped_[1]); float srz = sin(transformTobeMapped_[2]); float crz = cos(transformTobeMapped_[2]);
        for (int i=0; i<laserCloudSelNum; ++i) { /* ... Jacobian calculation ... */
            pointOri = laserCloudOri_->points[i]; coeff = coeffSel_->points[i];
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;
            float ary = ((cry*srx*srz - crz*sry)*pointOri.x + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x + ((-cry*crz - srx*sry*srz)*pointOri.x + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;
            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            matA.at<float>(i,0) = arx; matA.at<float>(i,1) = ary; matA.at<float>(i,2) = arz; matA.at<float>(i,3) = coeff.x; matA.at<float>(i,4) = coeff.y; matA.at<float>(i,5) = coeff.z; matB.at<float>(i,0) = -coeff.intensity;
        }
        cv::transpose(matA, matAt); matAtA = matAt * matA; matAtB = matAt * matB; cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
        if (iterCount == 0) { cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0)); cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0)); cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0)); cv::eigen(matAtA, matE, matV); matV.copyTo(matV2); isDegenerate_ = false; float eignThre[6] = {10,10,10,1,1,1}; for (int i=5; i>=0; --i) { if (matE.at<float>(0,i) < eignThre[i]) { for (int j=0; j<6; ++j) matV2.at<float>(i,j) = 0; isDegenerate_ = true; } else break; } matP_ = matV.inv() * matV2; }
        if (isDegenerate_) { cv::Mat matX2(6,1,CV_32F, cv::Scalar::all(0)); matX.copyTo(matX2); matX = matP_ * matX2; }
        transformTobeMapped_[0] += matX.at<float>(0,0); transformTobeMapped_[1] += matX.at<float>(1,0); transformTobeMapped_[2] += matX.at<float>(2,0); transformTobeMapped_[3] += matX.at<float>(3,0); transformTobeMapped_[4] += matX.at<float>(4,0); transformTobeMapped_[5] += matX.at<float>(5,0);
        deltaR_final = sqrt(pow(rad2deg(matX.at<float>(0,0)),2) + pow(rad2deg(matX.at<float>(1,0)),2) + pow(rad2deg(matX.at<float>(2,0)),2));
        deltaT_final = sqrt(pow(matX.at<float>(3,0)*100,2) + pow(matX.at<float>(4,0)*100,2) + pow(matX.at<float>(5,0)*100,2));
        if (deltaR_final < 0.05 && deltaT_final < 0.05) break;
      }
      last_icp_delta_rotation_ = deltaR_final; // Store for health
      last_icp_delta_translation_ = deltaT_final; // Store for health

      if (enable_health_warnings_param_) {
          if (last_icp_delta_rotation_ > max_icp_delta_rotation_deg_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: ICP delta rotation (%.2f deg) exceeds threshold (%.2f deg).", last_icp_delta_rotation_, max_icp_delta_rotation_deg_param_);
          if (last_icp_delta_translation_ > max_icp_delta_translation_cm_param_) RCLCPP_WARN(this->get_logger(), "LaserMapping: ICP delta translation (%.2f cm) exceeds threshold (%.2f cm).", last_icp_delta_translation_, max_icp_delta_translation_cm_param_);
          if (warn_on_icp_degeneracy_param_ && isDegenerate_) RCLCPP_WARN(this->get_logger(), "LaserMapping: ICP optimization was degenerate.");
      }
      if (markers_icp_corr_ && correspondence_marker_array.markers.size() > 0) pub_icp_correspondence_markers_->publish(correspondence_marker_array);
      if (markers_sel_features_ && selected_points_marker.points.size() > 0) { /* ... publish selected_feature_marker_array ... */ }
      transformUpdate();
    } else { transformUpdate(); }

    PointType pointSel;
    for (const auto& p_orig : *laserCloudCornerLast_) { /* ... add to map ... */
        pointAssociateToMap(&p_orig, &pointSel); int cubeI = static_cast<int>((pointSel.x + 25.0f) / 50.0f) + laserCloudCenWidth_; int cubeJ = static_cast<int>((pointSel.y + 25.0f) / 50.0f) + laserCloudCenHeight_; int cubeK = static_cast<int>((pointSel.z + 25.0f) / 50.0f) + laserCloudCenDepth_;
        if (pointSel.x + 25.0f < 0) cubeI--; if (pointSel.y + 25.0f < 0) cubeJ--; if (pointSel.z + 25.0f < 0) cubeK--;
        if (cubeI>=0&&cubeI<laserCloudWidth_ && cubeJ>=0&&cubeJ<laserCloudHeight_ && cubeK>=0&&cubeK<laserCloudDepth_) laserCloudCornerArray_[cubeI + laserCloudWidth_*cubeJ + laserCloudWidth_*laserCloudHeight_*cubeK]->push_back(pointSel);
    }
    for (const auto& p_orig : *laserCloudSurfLast_down_) { /* ... add to map ... */
        pointAssociateToMap(&p_orig, &pointSel); int cubeI = static_cast<int>((pointSel.x + 25.0f) / 50.0f) + laserCloudCenWidth_; int cubeJ = static_cast<int>((pointSel.y + 25.0f) / 50.0f) + laserCloudCenHeight_; int cubeK = static_cast<int>((pointSel.z + 25.0f) / 50.0f) + laserCloudCenDepth_;
        if (pointSel.x + 25.0f < 0) cubeI--; if (pointSel.y + 25.0f < 0) cubeJ--; if (pointSel.z + 25.0f < 0) cubeK--;
        if (cubeI>=0&&cubeI<laserCloudWidth_ && cubeJ>=0&&cubeJ<laserCloudHeight_ && cubeK>=0&&cubeK<laserCloudDepth_) laserCloudSurfArray_[cubeI + laserCloudWidth_*cubeJ + laserCloudWidth_*laserCloudHeight_*cubeK]->push_back(pointSel);
    }
    for (int i=0; i<laserCloudValidNum_; ++i) { /* ... downsample map cubes ... */
        int ind = laserCloudValidInd_[i]; laserCloudCornerArray2_[ind]->clear(); downSizeFilterCorner_.setInputCloud(laserCloudCornerArray_[ind]); downSizeFilterCorner_.filter(*laserCloudCornerArray2_[ind]); laserCloudCornerArray_[ind].swap(laserCloudCornerArray2_[ind]);
        laserCloudSurfArray2_[ind]->clear(); downSizeFilterSurf_.setInputCloud(laserCloudSurfArray_[ind]); downSizeFilterSurf_.filter(*laserCloudSurfArray2_[ind]); laserCloudSurfArray_[ind].swap(laserCloudSurfArray2_[ind]);
    }
    
    laserCloudSurround2_->clear(); laserCloudSurround2_corner_->clear();
    for (int i=0; i<laserCloudSurroundNum_; ++i) { int ind = laserCloudSurroundInd_[i]; *laserCloudSurround2_corner_ += *laserCloudCornerArray_[ind]; *laserCloudSurround2_ += *laserCloudSurfArray_[ind]; }
    sensor_msgs::msg::PointCloud2 tempCloudMsg; rclcpp::Time currentTime = rclcpp::Time(static_cast<uint64_t>(timeLaserCloudCornerLast_ * 1e9));
    pcl::toROSMsg(*laserCloudSurround2_, tempCloudMsg); tempCloudMsg.header.stamp = currentTime; tempCloudMsg.header.frame_id = "camera_init"; pubLaserCloudSurround_->publish(tempCloudMsg);
    pcl::toROSMsg(*laserCloudSurround2_corner_, tempCloudMsg); tempCloudMsg.header.stamp = currentTime; tempCloudMsg.header.frame_id = "camera_init"; pubLaserCloudSurroundCorner_->publish(tempCloudMsg);
    laserCloudFullResColor_->clear(); *laserCloudFullRes2_ = *laserCloudFullRes_;
    for (const auto& pt : laserCloudFullRes2_->points) { pcl::PointXYZRGB temp_rgb_pt; RGBpointAssociateToMap(&pt, &temp_rgb_pt); laserCloudFullResColor_->push_back(temp_rgb_pt); }
    pcl::toROSMsg(*laserCloudFullResColor_, tempCloudMsg); tempCloudMsg.header.stamp = currentTime; tempCloudMsg.header.frame_id = "camera_init"; pubLaserCloudFullRes_->publish(tempCloudMsg);
    *laserCloudFullResColor_pcd_ += *laserCloudFullResColor_;

    tf2::Quaternion q_aft_mapped; q_aft_mapped.setRPY(-transformAftMapped_[1], -transformAftMapped_[0], transformAftMapped_[2]);
    odomAftMapped_.header.stamp = currentTime; odomAftMapped_.pose.pose.orientation = tf2::toMsg(q_aft_mapped);
    odomAftMapped_.pose.pose.position.x = transformAftMapped_[3]; odomAftMapped_.pose.pose.position.y = transformAftMapped_[4]; odomAftMapped_.pose.pose.position.z = transformAftMapped_[5];
    pubOdomAftMapped_->publish(odomAftMapped_);
    geometry_msgs::msg::TransformStamped tfStamped; tfStamped.header.stamp = this->get_clock()->now(); tfStamped.header.frame_id = "camera_init"; tfStamped.child_frame_id = "aft_mapped";
    tfStamped.transform.translation.x = transformAftMapped_[3]; tfStamped.transform.translation.y = transformAftMapped_[4]; tfStamped.transform.translation.z = transformAftMapped_[5]; tfStamped.transform.rotation = tf2::toMsg(q_aft_mapped);
    tf_broadcaster_->sendTransform(tfStamped);
    kfNum_++;
    if (kfNum_ >= 20) { Eigen::Matrix<float,7,1> kf_pose_eigen; kf_pose_eigen << tf2::toMsg(q_aft_mapped).x, tf2::toMsg(q_aft_mapped).y, tf2::toMsg(q_aft_mapped).z, tf2::toMsg(q_aft_mapped).w, transformAftMapped_[3], transformAftMapped_[4], transformAftMapped_[5]; keyframe_pose_.push_back(kf_pose_eigen); kfNum_ = 0; }

    // Determine and publish LaserMapping health status
    loam_adaptive_parameter_manager::LaserMappingHealth current_health = loam_adaptive_parameter_manager::LaserMappingHealth::HEALTHY;

    bool low_downsampled_corner = static_cast<int>(laserCloudCornerLast_down_->size()) < min_downsampled_corner_features_param_;
    bool low_downsampled_surf = static_cast<int>(laserCloudSurfLast_down_->size()) < min_downsampled_surf_features_param_;
    bool low_map_corner_icp = static_cast<int>(laserCloudCornerFromMap_->points.size()) < min_map_corner_points_for_icp_param_;
    bool low_map_surf_icp = static_cast<int>(laserCloudSurfFromMap_->points.size()) < min_map_surf_points_for_icp_param_;

    // Use stored ICP iteration results for health
    bool low_icp_correspondences = last_icp_correspondences_ < min_icp_correspondences_param_;
    bool high_icp_delta_rot = last_icp_delta_rotation_ > max_icp_delta_rotation_deg_param_;
    bool high_icp_delta_trans = last_icp_delta_translation_ > max_icp_delta_translation_cm_param_;
    // isDegenerate_ is already a member and updated during ICP

    if (isDegenerate_ && warn_on_icp_degeneracy_param_) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::ICP_DEGENERATE;
    } else if (high_icp_delta_rot) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::HIGH_ICP_DELTA_ROTATION;
    } else if (high_icp_delta_trans) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::HIGH_ICP_DELTA_TRANSLATION;
    } else if (low_icp_correspondences && (laserCloudCornerFromMap_->points.size() > 10 && laserCloudSurfFromMap_->points.size() > 100) ) { // only if ICP was actually run
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::LOW_ICP_CORRESPONDENCES;
    } else if (low_map_corner_icp) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::LOW_MAP_CORNER_POINTS_ICP;
    } else if (low_map_surf_icp) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::LOW_MAP_SURF_POINTS_ICP;
    } else if (low_downsampled_corner) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::LOW_DOWNSAMPLED_CORNER_FEATURES;
    } else if (low_downsampled_surf) {
        current_health = loam_adaptive_parameter_manager::LaserMappingHealth::LOW_DOWNSAMPLED_SURF_FEATURES;
    }

    std_msgs::msg::Int32 health_msg;
    health_msg.data = static_cast<int>(current_health);
    if(pub_health_status_) { // Check if publisher is initialized
        pub_health_status_->publish(health_msg);
    }
  }

}; // LaserMapping Class End


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto laser_mapping_node = std::make_shared<LaserMapping>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(laser_mapping_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
