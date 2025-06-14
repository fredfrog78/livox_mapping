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
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
// #include <pcl/filters/voxel_grid.h> // Not used in the provided snippet
// #include <pcl/kdtree/kdtree_flann.h> // Not used in the provided snippet

#include <opencv2/opencv.hpp> // For cv::Mat, cv::eigen etc.
#include <eigen3/Eigen/Core> // For Eigen::Vector3d

#include <vector>
#include <cmath>
#include <algorithm> // For std::fill, std::floor

#include "adaptive_parameter_manager_types.h" // For health status enums
#include "std_msgs/msg/int32.hpp" // To publish enum as integer

typedef pcl::PointXYZI PointType;
const int MAX_CLOUD_SIZE = 32000; // Maximum cloud size to process

class ScanRegistration : public rclcpp::Node {
public:
  ScanRegistration() : Node("scan_registration_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing ScanRegistration Node");

    // Initialize publishers
    pubLaserCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_cloud", 20);
    pubCornerPointsSharp_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_sharp", 20);
    pubSurfPointsFlat_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_flat", 20);
    pub_health_status_ = this->create_publisher<std_msgs::msg::Int32>("/scan_registration/health_status", 10);
    
    // pubLaserCloud_temp_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_temp", 2); // For hkmars_data (commented out)


    // Initialize subscriber
    // Assuming the input topic is /livox_pcl0 from livox_repub node
    subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox_pcl0", rclcpp::QoS(100), // Original queue size was 100
        std::bind(&ScanRegistration::laserCloudHandler, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "--- ScanRegistration Node ---");
    if (subLaserCloud_) {
        RCLCPP_INFO(this->get_logger(), "Subscribed to PointCloud2 on topic: %s", subLaserCloud_->get_topic_name());
    }
    RCLCPP_INFO(this->get_logger(), "-----------------------------");

    // subLaserCloud_for_hk_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/livox/lidar", 2, std::bind(&ScanRegistration::laserCloudHandler_temp, this, std::placeholders::_1)); // For hkmars_data (commented out)


    // Initialize OpenCV matrices
    matA1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1_ = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    matV1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

    CloudFeatureFlag_.resize(MAX_CLOUD_SIZE);

    // Declare and get health monitoring parameters
    this->declare_parameter<bool>("health.enable_health_warnings", true);
    this->declare_parameter<int>("health.min_raw_points_for_feature_extraction", 10);
    this->declare_parameter<int>("health.min_sharp_features", 15);
    this->declare_parameter<int>("health.min_flat_features", 40);

    this->get_parameter("health.enable_health_warnings", enable_health_warnings_param_);
    this->get_parameter("health.min_raw_points_for_feature_extraction", min_raw_points_param_);
    this->get_parameter("health.min_sharp_features", min_sharp_features_param_);
    this->get_parameter("health.min_flat_features", min_flat_features_param_);

    RCLCPP_INFO(this->get_logger(), "Health Monitoring Parameters:");
    RCLCPP_INFO(this->get_logger(), "  enable_health_warnings: %s", enable_health_warnings_param_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  min_raw_points_for_feature_extraction: %d", min_raw_points_param_);
    RCLCPP_INFO(this->get_logger(), "  min_sharp_features: %d", min_sharp_features_param_);
    RCLCPP_INFO(this->get_logger(), "  min_flat_features: %d", min_flat_features_param_);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_health_status_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_temp_; // For hkmars_data

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_for_hk_; // For hkmars_data

  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> msg_window_; // For laserCloudHandler_temp

  cv::Mat matA1_;
  cv::Mat matD1_;
  cv::Mat matV1_;

  std::vector<int> CloudFeatureFlag_;
  int scanID_; // Member variable if needed across methods, or local if only in callback

  // Health monitoring parameters
  bool enable_health_warnings_param_;
  int min_raw_points_param_;
  int min_sharp_features_param_;
  int min_flat_features_param_;


  bool plane_judge(const std::vector<PointType>& point_list, const int plane_threshold) {
    int num = point_list.size();
    if (num == 0) return false;

    float cx = 0;
    float cy = 0;
    float cz = 0;
    for (int j = 0; j < num; j++) {
        cx += point_list[j].x;
        cy += point_list[j].y;
        cz += point_list[j].z;
    }
    cx /= num;
    cy /= num;
    cz /= num;
    
    float a11 = 0;
    float a12 = 0;
    float a13 = 0;
    float a22 = 0;
    float a23 = 0;
    float a33 = 0;
    for (int j = 0; j < num; j++) {
        float ax = point_list[j].x - cx;
        float ay = point_list[j].y - cy;
        float az = point_list[j].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
    }
    a11 /= num;
    a12 /= num;
    a13 /= num;
    a22 /= num;
    a23 /= num;
    a33 /= num;

    matA1_.at<float>(0, 0) = a11;
    matA1_.at<float>(0, 1) = a12;
    matA1_.at<float>(0, 2) = a13;
    matA1_.at<float>(1, 0) = a12;
    matA1_.at<float>(1, 1) = a22;
    matA1_.at<float>(1, 2) = a23;
    matA1_.at<float>(2, 0) = a13;
    matA1_.at<float>(2, 1) = a23;
    matA1_.at<float>(2, 2) = a33;

    cv::eigen(matA1_, matD1_, matV1_); // Output eigenvalues to matD1_, eigenvectors to matV1_
    
    // Check if matD1_ has at least 2 elements before accessing them
    if (matD1_.rows >= 1 && matD1_.cols >= 2) {
        if (matD1_.at<float>(0, 0) > plane_threshold * matD1_.at<float>(0, 1)) {
            return true;
        }
    } else if (matD1_.rows >= 2 && matD1_.cols >=1) { // if it's a column vector
        if (matD1_.at<float>(0, 0) > plane_threshold * matD1_.at<float>(1, 0)) {
            return true;
        }
    }
    return false;
  }

  // Commented out as it was in the original main. Refactored for ROS2 if needed.
  /*
  void laserCloudHandler_temp(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg) //for hkmars data
  {
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

    if(msg_window_.size() < 2){
      msg_window_.push_back(laserCloudMsg);
    }
    else{
      msg_window_.erase(msg_window_.begin());
      msg_window_.push_back(laserCloudMsg);
    }

    for(size_t i = 0; i < msg_window_.size();i++){
      pcl::PointCloud<PointType> temp;
      pcl::fromROSMsg(*msg_window_[i], temp);
      *laserCloudIn += temp;
    }
    sensor_msgs::msg::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloudIn, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/livox"; // Consider making this a parameter
    pubLaserCloud_temp_->publish(laserCloudOutMsg);
  }
  */

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg) {
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    int cloudSize = laserCloudIn.points.size();

    if (cloudSize > MAX_CLOUD_SIZE) {
        RCLCPP_WARN(this->get_logger(), "Cloud size %d exceeds MAX_CLOUD_SIZE %d. Truncating.", cloudSize, MAX_CLOUD_SIZE);
        cloudSize = MAX_CLOUD_SIZE;
    }
    if (cloudSize == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
        return;
    }
    
    PointType point;
    pcl::PointCloud<PointType> Allpoints;
    Allpoints.reserve(cloudSize);

    for (int i = 0; i < cloudSize; i++) {
      point.x = laserCloudIn.points[i].x;
      point.y = laserCloudIn.points[i].y;
      point.z = laserCloudIn.points[i].z;
      
      // Directly use intensity from input (from livox_repub)
      // Curvature is no longer used as timestamp; timestamp is part of PointCloud2 header
      point.intensity = laserCloudIn.points[i].intensity; 

      // scanID_ calculation removed, as line info is now in intensity
      // double theta = std::atan2(laserCloudIn.points[i].y, laserCloudIn.points[i].z) / M_PI * 180 + 180;
      // scanID_ = std::floor(theta / 9); 


      if (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      Allpoints.push_back(point);
    }

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    *laserCloud = Allpoints; // Use the processed points
    cloudSize = laserCloud->size(); // Update cloudSize after filtering invalid points

    // This is the cloudSize after NaN filtering. This is the best place to check min_raw_points_param_
    // for the purpose of the ScanRegistrationHealth::LOW_RAW_POINTS state.
    bool raw_points_low_for_health_status = (cloudSize < min_raw_points_param_);


    if (enable_health_warnings_param_ && cloudSize < min_raw_points_param_) {
        RCLCPP_WARN(this->get_logger(), "Not enough raw points (%d) for feature extraction after filtering. Minimum required: %d", cloudSize, min_raw_points_param_);
        // If too few points even for health check, publish bad health and return.
        if (pub_health_status_) {
            std_msgs::msg::Int32 health_msg;
            health_msg.data = static_cast<int>(loam_adaptive_parameter_manager::ScanRegistrationHealth::LOW_RAW_POINTS);
            pub_health_status_->publish(health_msg);
        }
        return;
    } else if (!enable_health_warnings_param_ && cloudSize < min_raw_points_param_){
        RCLCPP_DEBUG(this->get_logger(), "Not enough raw points (%d) for feature extraction after filtering (minimum required: %d), but health warnings are disabled.", cloudSize, min_raw_points_param_);
        if (cloudSize < 6) { // Absolute minimum to proceed
             RCLCPP_ERROR(this->get_logger(), "Critically low number of points (%d) after filtering, even with health warnings disabled. Cannot safely proceed.", cloudSize);
             if (pub_health_status_) {
                 std_msgs::msg::Int32 health_msg;
                 health_msg.data = static_cast<int>(loam_adaptive_parameter_manager::ScanRegistrationHealth::LOW_RAW_POINTS);
                 pub_health_status_->publish(health_msg);
             }
             return;
        }
    }
    
    std::fill(CloudFeatureFlag_.begin(), CloudFeatureFlag_.begin() + cloudSize, 0);
        
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> surfPointsFlat;

    int debugnum1 = 0;
    int debugnum2 = 0;
    int debugnum3 = 0;
    int debugnum4 = 0;
    int debugnum5 = 0;

    int count_num = 1;
    bool left_surf_flag = false;
    bool right_surf_flag = false;

    for (int i = 5; i < cloudSize - 5; i += count_num ) {
      float ldiffX = laserCloud->points[i - 4].x + laserCloud->points[i - 3].x
                   - 4 * laserCloud->points[i - 2].x
                   + laserCloud->points[i - 1].x + laserCloud->points[i].x;
      float ldiffY = laserCloud->points[i - 4].y + laserCloud->points[i - 3].y
                   - 4 * laserCloud->points[i - 2].y
                   + laserCloud->points[i - 1].y + laserCloud->points[i].y;
      float ldiffZ = laserCloud->points[i - 4].z + laserCloud->points[i - 3].z
                   - 4 * laserCloud->points[i - 2].z
                   + laserCloud->points[i - 1].z + laserCloud->points[i].z;
      float left_curvature = ldiffX * ldiffX + ldiffY * ldiffY + ldiffZ * ldiffZ;

      if (left_curvature < 0.01) {
        std::vector<PointType> left_list;
        left_list.reserve(4);
        for(int j = -4; j < 0; j++){
          left_list.push_back(laserCloud->points[i+j]);
        }
        if (left_curvature < 0.01) CloudFeatureFlag_[i-2] = 1; // surf point flag
        left_surf_flag = true;
      } else {
        left_surf_flag = false;
      }

      float rdiffX = laserCloud->points[i + 4].x + laserCloud->points[i + 3].x
                   - 4 * laserCloud->points[i + 2].x
                   + laserCloud->points[i + 1].x + laserCloud->points[i].x;
      float rdiffY = laserCloud->points[i + 4].y + laserCloud->points[i + 3].y
                   - 4 * laserCloud->points[i + 2].y
                   + laserCloud->points[i + 1].y + laserCloud->points[i].y;
      float rdiffZ = laserCloud->points[i + 4].z + laserCloud->points[i + 3].z
                   - 4 * laserCloud->points[i + 2].z
                   + laserCloud->points[i + 1].z + laserCloud->points[i].z;
      float right_curvature = rdiffX * rdiffX + rdiffY * rdiffY + rdiffZ * rdiffZ;

      if (right_curvature < 0.01) {
        std::vector<PointType> right_list;
        right_list.reserve(4);
        for(int j = 1; j < 5; j++){
          right_list.push_back(laserCloud->points[i+j]);
        }
        if (right_curvature < 0.01) CloudFeatureFlag_[i+2] = 1; // surf point flag
        count_num = 4;
        right_surf_flag = true;
      } else {
        count_num = 1;
        right_surf_flag = false;
      }

      if (left_surf_flag && right_surf_flag) {
        debugnum4++;
        Eigen::Vector3d norm_left(0,0,0);
        Eigen::Vector3d norm_right(0,0,0);
        for(int k = 1;k<5;k++){
            Eigen::Vector3d tmp(laserCloud->points[i-k].x-laserCloud->points[i].x,
                               laserCloud->points[i-k].y-laserCloud->points[i].y,
                               laserCloud->points[i-k].z-laserCloud->points[i].z);
            tmp.normalize();
            norm_left += (k/10.0)* tmp;
        }
        for(int k = 1;k<5;k++){
            Eigen::Vector3d tmp(laserCloud->points[i+k].x-laserCloud->points[i].x,
                               laserCloud->points[i+k].y-laserCloud->points[i].y,
                               laserCloud->points[i+k].z-laserCloud->points[i].z);
            tmp.normalize();
            norm_right += (k/10.0)* tmp;
        }

        double norm_left_mag = norm_left.norm();
        double norm_right_mag = norm_right.norm();
        double cc = 0.0;
        if (norm_left_mag > 1e-6 && norm_right_mag > 1e-6) {
             cc = std::abs(norm_left.dot(norm_right) / (norm_left_mag * norm_right_mag));
        }

        Eigen::Vector3d last_tmp(laserCloud->points[i-4].x-laserCloud->points[i].x,
                                 laserCloud->points[i-4].y-laserCloud->points[i].y,
                                 laserCloud->points[i-4].z-laserCloud->points[i].z);
        Eigen::Vector3d current_tmp(laserCloud->points[i+4].x-laserCloud->points[i].x,
                                    laserCloud->points[i+4].y-laserCloud->points[i].y,
                                    laserCloud->points[i+4].z-laserCloud->points[i].z);
        double last_dis = last_tmp.norm();
        double current_dis = current_tmp.norm();

        if (cc < 0.5 && last_dis > 0.05 && current_dis > 0.05 ) {
          debugnum5++;
          CloudFeatureFlag_[i] = 150;
        }
      }
    }

    for (int i = 5; i < cloudSize - 5; i++) {
      float diff_left[2];
      float diff_right[2];
      float depth = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                         laserCloud->points[i].y * laserCloud->points[i].y +
                         laserCloud->points[i].z * laserCloud->points[i].z);

      for(int count = 1; count < 3; count++ ){
        float diffX1 = laserCloud->points[i + count].x - laserCloud->points[i].x;
        float diffY1 = laserCloud->points[i + count].y - laserCloud->points[i].y;
        float diffZ1 = laserCloud->points[i + count].z - laserCloud->points[i].z;
        diff_right[count - 1] = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);

        float diffX2 = laserCloud->points[i - count].x - laserCloud->points[i].x;
        float diffY2 = laserCloud->points[i - count].y - laserCloud->points[i].y;
        float diffZ2 = laserCloud->points[i - count].z - laserCloud->points[i].z;
        diff_left[count - 1] = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
      }
      
      float depth_right = Eigen::Vector3f(laserCloud->points[i+1].x, laserCloud->points[i+1].y, laserCloud->points[i+1].z).norm();
      float depth_left  = Eigen::Vector3f(laserCloud->points[i-1].x, laserCloud->points[i-1].y, laserCloud->points[i-1].z).norm();


      if ((diff_right[0] > 0.1 * depth && diff_left[0] > 0.1 * depth) ) {
        debugnum1++;  
        CloudFeatureFlag_[i] = 250; // outlier flag
        continue;
      }

      if (std::abs(diff_right[0] - diff_left[0]) > 0.1) {
        if (diff_right[0] > diff_left[0]) {
          Eigen::Vector3d surf_vector(laserCloud->points[i-4].x-laserCloud->points[i].x,
                                      laserCloud->points[i-4].y-laserCloud->points[i].y,
                                      laserCloud->points[i-4].z-laserCloud->points[i].z);
          Eigen::Vector3d lidar_vector(laserCloud->points[i].x,
                                       laserCloud->points[i].y,
                                       laserCloud->points[i].z);
          double left_surf_dis = surf_vector.norm();
          double cc = 0.0;
          if (left_surf_dis > 1e-6 && lidar_vector.norm() > 1e-6) {
            cc = std::abs(surf_vector.dot(lidar_vector) / (left_surf_dis * lidar_vector.norm()));
          }
          
          std::vector<PointType> left_list; left_list.reserve(4);
          double min_dis = 10000, max_dis = 0;
          for(int j = 0; j < 4; j++){
            left_list.push_back(laserCloud->points[i-j]);
            if (j == 3) break;
            Eigen::Vector3d temp_vector(laserCloud->points[i-j].x-laserCloud->points[i-j-1].x,
                                        laserCloud->points[i-j].y-laserCloud->points[i-j-1].y,
                                        laserCloud->points[i-j].z-laserCloud->points[i-j-1].z);
            double temp_dis = temp_vector.norm();
            if(temp_dis < min_dis) min_dis = temp_dis;
            if(temp_dis > max_dis) max_dis = temp_dis;
          }
          bool left_is_plane = plane_judge(left_list,100);

          if (left_is_plane && (max_dis < 2*min_dis) && left_surf_dis < 0.05 * depth  && cc < 0.8){
            if (depth_right > depth_left) CloudFeatureFlag_[i] = 100;
            else if (depth_right == 0) CloudFeatureFlag_[i] = 100;
          }
        } else {
          Eigen::Vector3d surf_vector(laserCloud->points[i+4].x-laserCloud->points[i].x,
                                      laserCloud->points[i+4].y-laserCloud->points[i].y,
                                      laserCloud->points[i+4].z-laserCloud->points[i].z);
          Eigen::Vector3d lidar_vector(laserCloud->points[i].x,
                                       laserCloud->points[i].y,
                                       laserCloud->points[i].z);
          double right_surf_dis = surf_vector.norm();
          double cc = 0.0;
          if (right_surf_dis > 1e-6 && lidar_vector.norm() > 1e-6) {
            cc = std::abs(surf_vector.dot(lidar_vector) / (right_surf_dis * lidar_vector.norm()));
          }

          std::vector<PointType> right_list; right_list.reserve(4);
          double min_dis = 10000, max_dis = 0;
          for(int j = 0; j < 4; j++){
            right_list.push_back(laserCloud->points[i+j]);
            if (j == 3) break;
            Eigen::Vector3d temp_vector(laserCloud->points[i+j].x-laserCloud->points[i+j+1].x,
                                        laserCloud->points[i+j].y-laserCloud->points[i+j+1].y,
                                        laserCloud->points[i+j].z-laserCloud->points[i+j+1].z);
            double temp_dis = temp_vector.norm();
            if(temp_dis < min_dis) min_dis = temp_dis;
            if(temp_dis > max_dis) max_dis = temp_dis;
          }
          bool right_is_plane = plane_judge(right_list,100);

          if (right_is_plane && (max_dis < 2*min_dis) && right_surf_dis < 0.05 * depth && cc < 0.8){
            if (depth_right < depth_left) CloudFeatureFlag_[i] = 100;
            else if (depth_left == 0) CloudFeatureFlag_[i] = 100;
          }
        }
      }

      if (CloudFeatureFlag_[i] == 100) {
        debugnum2++;
        Eigen::Vector3d norm_front(0,0,0);
        Eigen::Vector3d norm_back(0,0,0);
        for(int k = 1;k<4;k++){
            Eigen::Vector3d tmp(laserCloud->points[i-k].x-laserCloud->points[i].x,
                                laserCloud->points[i-k].y-laserCloud->points[i].y,
                                laserCloud->points[i-k].z-laserCloud->points[i].z);
            tmp.normalize();
            norm_front += (k/6.0)* tmp;
        }
        for(int k = 1;k<4;k++){
            Eigen::Vector3d tmp(laserCloud->points[i+k].x-laserCloud->points[i].x,
                                laserCloud->points[i+k].y-laserCloud->points[i].y,
                                laserCloud->points[i+k].z-laserCloud->points[i].z);
            tmp.normalize();
            norm_back += (k/6.0)* tmp;
        }
        
        double norm_front_mag = norm_front.norm();
        double norm_back_mag = norm_back.norm();
        double cc = 0.0;
        if (norm_front_mag > 1e-6 && norm_back_mag > 1e-6) {
            cc = std::abs(norm_front.dot(norm_back) / (norm_front_mag * norm_back_mag));
        }

        if (cc < 0.8) {
          debugnum3++;
        } else {
          CloudFeatureFlag_[i] = 0;
        }
        continue;
      }
    }

    for (int i = 0; i < cloudSize; i++) {
      if (CloudFeatureFlag_[i] == 250) {
        continue;
      }

      if (CloudFeatureFlag_[i] == 100 || CloudFeatureFlag_[i] == 150) {
        cornerPointsSharp.push_back(laserCloud->points[i]);
      } else if (CloudFeatureFlag_[i] == 1 || CloudFeatureFlag_[i] == 0) {
        surfPointsFlat.push_back(laserCloud->points[i]);
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "ALL point: %d, outliers: %d", cloudSize, debugnum1);
    RCLCPP_DEBUG(this->get_logger(), "break points: %d, break feature: %d", debugnum2, debugnum3);
    RCLCPP_DEBUG(this->get_logger(), "normal points: %d, surf-surf feature: %d", debugnum4, debugnum5);

    int num_sharp_features = cornerPointsSharp.size();
    int num_flat_features = surfPointsFlat.size();

    if (enable_health_warnings_param_) {
        if (num_sharp_features < min_sharp_features_param_) {
            RCLCPP_WARN(this->get_logger(), "Number of sharp features (%d) is below threshold (%d).", num_sharp_features, min_sharp_features_param_);
        }
        if (num_flat_features < min_flat_features_param_) {
            RCLCPP_WARN(this->get_logger(), "Number of flat features (%d) is below threshold (%d).", num_flat_features, min_flat_features_param_);
        }
    }

    sensor_msgs::msg::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header = laserCloudMsg->header;
    pubLaserCloud_->publish(laserCloudOutMsg);

    sensor_msgs::msg::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header = laserCloudMsg->header;
    pubCornerPointsSharp_->publish(cornerPointsSharpMsg);

    sensor_msgs::msg::PointCloud2 surfPointsFlatMsg;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlatMsg);
    surfPointsFlatMsg.header = laserCloudMsg->header;
    pubSurfPointsFlat_->publish(surfPointsFlatMsg);

    // Determine and publish health status
    loam_adaptive_parameter_manager::ScanRegistrationHealth current_health = loam_adaptive_parameter_manager::ScanRegistrationHealth::HEALTHY;
    // raw_points_low_for_health_status is already determined after initial filtering and before the return statements for low points
    bool sharp_features_low = (num_sharp_features < min_sharp_features_param_);
    bool flat_features_low = (num_flat_features < min_flat_features_param_);

    if (raw_points_low_for_health_status) { // This reflects points *before* feature extraction logic
        current_health = loam_adaptive_parameter_manager::ScanRegistrationHealth::LOW_RAW_POINTS;
    } else if (sharp_features_low && flat_features_low) {
        current_health = loam_adaptive_parameter_manager::ScanRegistrationHealth::LOW_SHARP_FEATURES;
    } else if (sharp_features_low) {
        current_health = loam_adaptive_parameter_manager::ScanRegistrationHealth::LOW_SHARP_FEATURES;
    } else if (flat_features_low) {
        current_health = loam_adaptive_parameter_manager::ScanRegistrationHealth::LOW_FLAT_FEATURES;
    }
    // else it remains HEALTHY

    std_msgs::msg::Int32 health_msg;
    health_msg.data = static_cast<int>(current_health);
    if (pub_health_status_) {
        pub_health_status_->publish(health_msg);
    }
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanRegistration>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
