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

#include <opencv2/opencv.hpp> // For cv::Mat, cv::eigen etc.
#include <eigen3/Eigen/Core> // For Eigen::Vector3d

#include <vector>
#include <cmath>
#include <algorithm> // For std::fill

// Note: Original code used PointXYZINormal for this file
typedef pcl::PointXYZINormal PointType; 
const int MAX_CLOUD_SIZE_HORIZON = 32000; // Maximum cloud size to process for this node

class ScanRegistrationHorizon : public rclcpp::Node {
public:
  ScanRegistrationHorizon() : Node("scan_registration_horizon_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing ScanRegistrationHorizon Node");

    // Declare and get parameters
    this->declare_parameter<int>("n_scans", 6);
    this->get_parameter("n_scans", N_SCANS_);
    RCLCPP_INFO(this->get_logger(), "Using N_SCANS: %d", N_SCANS_);


    // Initialize publishers
    pubLaserCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_cloud_horizon", 20); // Topic renamed for clarity
    pubCornerPointsSharp_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_sharp_horizon", 20);
    pubSurfPointsFlat_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_flat_horizon", 20);
    
    // pubLaserCloud_temp_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_temp_horizon", 2); // For hkmars_data (commented out)

    // Initialize subscriber
    // Assuming the input topic is /livox_pcl0 from livox_repub node
    subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox_pcl0", rclcpp::QoS(100), 
        std::bind(&ScanRegistrationHorizon::laserCloudHandler, this, std::placeholders::_1));

    // subLaserCloud_for_hk_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/livox/lidar", 2, std::bind(&ScanRegistrationHorizon::laserCloudHandler_temp, this, std::placeholders::_1));

    // Initialize OpenCV matrices
    matA1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1_ = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    matV1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

    CloudFeatureFlag_.resize(MAX_CLOUD_SIZE_HORIZON);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_temp_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_for_hk_;

  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> msg_window_; 

  cv::Mat matA1_;
  cv::Mat matD1_;
  cv::Mat matV1_;

  std::vector<int> CloudFeatureFlag_;
  int N_SCANS_; // Number of scan lines, parameterizable
  // int scanID_ ; // Not needed as member if only used locally in laserCloudHandler


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
    
    float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
    for (int j = 0; j < num; j++) {
        float ax = point_list[j].x - cx;
        float ay = point_list[j].y - cy;
        float az = point_list[j].z - cz;
        a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
        a22 += ay * ay; a23 += ay * az; a33 += az * az;
    }
    a11 /= num; a12 /= num; a13 /= num; a22 /= num; a23 /= num; a33 /= num;

    matA1_.at<float>(0, 0) = a11; matA1_.at<float>(0, 1) = a12; matA1_.at<float>(0, 2) = a13;
    matA1_.at<float>(1, 0) = a12; matA1_.at<float>(1, 1) = a22; matA1_.at<float>(1, 2) = a23;
    matA1_.at<float>(2, 0) = a13; matA1_.at<float>(2, 1) = a23; matA1_.at<float>(2, 2) = a33;

    cv::eigen(matA1_, matD1_, matV1_);
    
    if (matD1_.rows >= 1 && matD1_.cols >= 2) { // Eigenvalues are in the first row
        if (matD1_.at<float>(0, 0) > plane_threshold * matD1_.at<float>(0, 1)) return true;
    } else if (matD1_.rows >= 2 && matD1_.cols >=1) { // Eigenvalues are in the first col
         if (matD1_.at<float>(0, 0) > plane_threshold * matD1_.at<float>(1, 0)) return true;
    }
    return false;
  }

  // Commented out as it was in the original main. Refactored for ROS2 if needed.
  /*
  void laserCloudHandler_temp(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
  {
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

    if(msg_window_.size() < 2){ // Assuming TO_MERGE_CNT is 2 for this handler
      msg_window_.push_back(laserCloudMsg);
    } else {
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
    laserCloudOutMsg.header.frame_id = "/livox"; // Consider parameterizing
    pubLaserCloud_temp_->publish(laserCloudOutMsg);
  }
  */

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg) {
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    int cloudSize = laserCloudIn.points.size();
    RCLCPP_DEBUG(this->get_logger(), "Received cloud for horizon processing with size: %d", cloudSize);

    if (cloudSize > MAX_CLOUD_SIZE_HORIZON) {
        RCLCPP_WARN(this->get_logger(), "Cloud size %d exceeds MAX_CLOUD_SIZE_HORIZON %d. Truncating.", cloudSize, MAX_CLOUD_SIZE_HORIZON);
        cloudSize = MAX_CLOUD_SIZE_HORIZON;
    }
     if (cloudSize == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud for horizon.");
        return;
    }

    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS_);
    for (int i = 0; i < N_SCANS_; ++i) {
        laserCloudScans[i].clear(); // Clear points from previous scan
        laserCloudScans[i].reserve(cloudSize / N_SCANS_); // Pre-allocate memory
    }

    for (int i = 0; i < cloudSize; i++) {
      point.x = laserCloudIn.points[i].x;
      point.y = laserCloudIn.points[i].y;
      point.z = laserCloudIn.points[i].z;
      // For PointXYZINormal, intensity and normal fields are available.
      // Original code used intensity to store scanID for N_SCANS=6.
      // And curvature was also copied.
      point.intensity = laserCloudIn.points[i].intensity; 
      point.curvature = laserCloudIn.points[i].curvature; 
      
      int scanID = 0;
      // This logic assumes point.intensity carries the scan ring ID.
      // This is a common practice with some Velodyne drivers, but for Livox,
      // the `livox_repub` node prepares intensity as line_number + reflectivity_fraction
      // and curvature as a normalized timestamp.
      // If N_SCANS_ is used, the input cloud *must* have intensity correctly populated
      // to represent the scan ring/line ID for this logic to work.
      if (N_SCANS_ > 1) { // Only try to get scanID if N_SCANS_ suggests multiple scans
          scanID = static_cast<int>(point.intensity); 
      }

      if (scanID >= 0 && scanID < N_SCANS_) {
        laserCloudScans[scanID].push_back(point);
      } else if (N_SCANS_ == 1) { // If N_SCANS is 1, put all points in the first scan
        laserCloudScans[0].push_back(point);
      }
      // else: point is dropped if scanID is out of expected range for N_SCANS > 1
    }

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    laserCloud->reserve(cloudSize); // Pre-allocate
    for (int i = 0; i < N_SCANS_; i++) {
      *laserCloud += laserCloudScans[i];
    }

    cloudSize = laserCloud->size();
    if (cloudSize < 10) { // Need at least ~10 points for feature extraction logic (i +/- 5)
        RCLCPP_WARN(this->get_logger(), "Not enough points (%d) for feature extraction after scan separation.", cloudSize);
        return;
    }
    
    std::fill(CloudFeatureFlag_.begin(), CloudFeatureFlag_.begin() + cloudSize, 0);
        
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> surfPointsFlat;

    int debugnum1 = 0, debugnum2 = 0, debugnum3 = 0, debugnum4 = 0, debugnum5 = 0;
    int count_num = 1;
    bool left_surf_flag = false, right_surf_flag = false;

    for (int i = 5; i < cloudSize - 5; i += count_num ) {
      // float depth = (Eigen::Vector3f(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z)).norm();

      float ldiffX = laserCloud->points[i-4].x + laserCloud->points[i-3].x - 4*laserCloud->points[i-2].x + laserCloud->points[i-1].x + laserCloud->points[i].x;
      float ldiffY = laserCloud->points[i-4].y + laserCloud->points[i-3].y - 4*laserCloud->points[i-2].y + laserCloud->points[i-1].y + laserCloud->points[i].y;
      float ldiffZ = laserCloud->points[i-4].z + laserCloud->points[i-3].z - 4*laserCloud->points[i-2].z + laserCloud->points[i-1].z + laserCloud->points[i].z;
      float left_curvature = ldiffX*ldiffX + ldiffY*ldiffY + ldiffZ*ldiffZ;

      if (left_curvature < 0.01) {
        std::vector<PointType> left_list; left_list.reserve(4);
        for(int j = -4; j < 0; j++) left_list.push_back(laserCloud->points[i+j]);
        if (left_curvature < 0.001 && plane_judge(left_list,1000)) CloudFeatureFlag_[i-2] = 1; 
        left_surf_flag = true;
      } else {
        left_surf_flag = false;
      }

      float rdiffX = laserCloud->points[i+4].x + laserCloud->points[i+3].x - 4*laserCloud->points[i+2].x + laserCloud->points[i+1].x + laserCloud->points[i].x;
      float rdiffY = laserCloud->points[i+4].y + laserCloud->points[i+3].y - 4*laserCloud->points[i+2].y + laserCloud->points[i+1].y + laserCloud->points[i].y;
      float rdiffZ = laserCloud->points[i+4].z + laserCloud->points[i+3].z - 4*laserCloud->points[i+2].z + laserCloud->points[i+1].z + laserCloud->points[i].z;
      float right_curvature = rdiffX*rdiffX + rdiffY*rdiffY + rdiffZ*rdiffZ;

      if (right_curvature < 0.01) {
        std::vector<PointType> right_list; right_list.reserve(4);
        for(int j = 1; j < 5; j++) right_list.push_back(laserCloud->points[i+j]);
        if (right_curvature < 0.001 && plane_judge(right_list,1000)) CloudFeatureFlag_[i+2] = 1;
        count_num = 4;
        right_surf_flag = true;
      } else {
        count_num = 1;
        right_surf_flag = false;
      }

      if (left_surf_flag && right_surf_flag) {
        debugnum4++;
        Eigen::Vector3d norm_left(0,0,0), norm_right(0,0,0);
        for(int k=1; k<5; ++k) {
          Eigen::Vector3d tmp(laserCloud->points[i-k].x-laserCloud->points[i].x, laserCloud->points[i-k].y-laserCloud->points[i].y, laserCloud->points[i-k].z-laserCloud->points[i].z);
          if (tmp.norm() > 1e-6) tmp.normalize(); norm_left += (k/10.0)*tmp;
        }
        for(int k=1; k<5; ++k) {
          Eigen::Vector3d tmp(laserCloud->points[i+k].x-laserCloud->points[i].x, laserCloud->points[i+k].y-laserCloud->points[i].y, laserCloud->points[i+k].z-laserCloud->points[i].z);
          if (tmp.norm() > 1e-6) tmp.normalize(); norm_right += (k/10.0)*tmp;
        }
        
        double cc = 0.0;
        if (norm_left.norm() > 1e-6 && norm_right.norm() > 1e-6) cc = std::abs(norm_left.dot(norm_right) / (norm_left.norm()*norm_right.norm()));
        
        Eigen::Vector3d last_tmp(laserCloud->points[i-4].x-laserCloud->points[i].x, laserCloud->points[i-4].y-laserCloud->points[i].y, laserCloud->points[i-4].z-laserCloud->points[i].z);
        Eigen::Vector3d current_tmp(laserCloud->points[i+4].x-laserCloud->points[i].x, laserCloud->points[i+4].y-laserCloud->points[i].y, laserCloud->points[i+4].z-laserCloud->points[i].z);
        double last_dis = last_tmp.norm();
        double current_dis = current_tmp.norm();

        if (cc < 0.5 && last_dis > 0.05 && current_dis > 0.05 ) CloudFeatureFlag_[i] = 150;
      }
    }

    for (int i = 5; i < cloudSize - 5; i++) {
      float diff_left[2], diff_right[2];
      float depth = (Eigen::Vector3f(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z)).norm();

      for(int count = 1; count < 3; count++ ){
        Eigen::Vector3f pt_curr(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z);
        Eigen::Vector3f pt_right(laserCloud->points[i+count].x, laserCloud->points[i+count].y, laserCloud->points[i+count].z);
        Eigen::Vector3f pt_left(laserCloud->points[i-count].x, laserCloud->points[i-count].y, laserCloud->points[i-count].z);
        diff_right[count-1] = (pt_right - pt_curr).norm();
        diff_left[count-1] = (pt_left - pt_curr).norm();
      }

      float depth_right = (Eigen::Vector3f(laserCloud->points[i+1].x, laserCloud->points[i+1].y, laserCloud->points[i+1].z)).norm();
      float depth_left  = (Eigen::Vector3f(laserCloud->points[i-1].x, laserCloud->points[i-1].y, laserCloud->points[i-1].z)).norm();
      
      if ((diff_right[0] > 0.1*depth && diff_left[0] > 0.1*depth) ) {
        debugnum1++; CloudFeatureFlag_[i] = 250; continue;
      }

      if (std::abs(diff_right[0] - diff_left[0]) > 0.1) {
        if (diff_right[0] > diff_left[0]) {
          Eigen::Vector3d surf_vector(laserCloud->points[i-4].x-laserCloud->points[i].x, laserCloud->points[i-4].y-laserCloud->points[i].y, laserCloud->points[i-4].z-laserCloud->points[i].z);
          Eigen::Vector3d lidar_vector(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z);
          double left_surf_dis = surf_vector.norm();
          double cc = 0.0;
          if (left_surf_dis > 1e-6 && lidar_vector.norm() > 1e-6) cc = std::abs(surf_vector.dot(lidar_vector) / (left_surf_dis*lidar_vector.norm()));
          
          std::vector<PointType> left_list; left_list.reserve(4);
          double min_dis = 10000, max_dis = 0;
          for(int j=0; j<4; ++j) {
            left_list.push_back(laserCloud->points[i-j]);
            if (j==3) break;
            Eigen::Vector3d tmp_v(laserCloud->points[i-j].x-laserCloud->points[i-j-1].x, laserCloud->points[i-j].y-laserCloud->points[i-j-1].y, laserCloud->points[i-j].z-laserCloud->points[i-j-1].z);
            double tmp_d = tmp_v.norm(); if(tmp_d < min_dis) min_dis = tmp_d; if(tmp_d > max_dis) max_dis = tmp_d;
          }
          bool left_is_plane = plane_judge(left_list,100);
          if (left_is_plane && (max_dis < 2*min_dis) && left_surf_dis < 0.05*depth && cc < 0.8) {
            if (depth_right > depth_left) CloudFeatureFlag_[i] = 100;
            else if (depth_right == 0) CloudFeatureFlag_[i] = 100;
          }
        } else {
          Eigen::Vector3d surf_vector(laserCloud->points[i+4].x-laserCloud->points[i].x, laserCloud->points[i+4].y-laserCloud->points[i].y, laserCloud->points[i+4].z-laserCloud->points[i].z);
          Eigen::Vector3d lidar_vector(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z);
          double right_surf_dis = surf_vector.norm();
          double cc = 0.0;
          if (right_surf_dis > 1e-6 && lidar_vector.norm() > 1e-6) cc = std::abs(surf_vector.dot(lidar_vector) / (right_surf_dis*lidar_vector.norm()));
          
          std::vector<PointType> right_list; right_list.reserve(4);
          double min_dis = 10000, max_dis = 0;
          for(int j=0; j<4; ++j) {
            // Original code: right_list.push_back(laserCloud->points[i-j]); This seems to be a copy-paste error from the left_list logic.
            // Should be points on the right side: laserCloud->points[i+j]
            right_list.push_back(laserCloud->points[i+j]); 
            if (j==3) break;
            Eigen::Vector3d tmp_v(laserCloud->points[i+j].x-laserCloud->points[i+j+1].x, laserCloud->points[i+j].y-laserCloud->points[i+j+1].y, laserCloud->points[i+j].z-laserCloud->points[i+j+1].z);
            double tmp_d = tmp_v.norm(); if(tmp_d < min_dis) min_dis = tmp_d; if(tmp_d > max_dis) max_dis = tmp_d;
          }
          bool right_is_plane = plane_judge(right_list,100);
          if (right_is_plane && (max_dis < 2*min_dis) && right_surf_dis < 0.05*depth && cc < 0.8) {
            if (depth_right < depth_left) CloudFeatureFlag_[i] = 100;
            else if (depth_left == 0) CloudFeatureFlag_[i] = 100;
          }
        }
      }

      if (CloudFeatureFlag_[i] == 100) {
        debugnum2++;
        Eigen::Vector3d norm_front(0,0,0), norm_back(0,0,0);
        for(int k=1; k<4; ++k) {
          Eigen::Vector3d tmp(laserCloud->points[i-k].x-laserCloud->points[i].x, laserCloud->points[i-k].y-laserCloud->points[i].y, laserCloud->points[i-k].z-laserCloud->points[i].z);
          if (tmp.norm() > 1e-6) tmp.normalize(); norm_front += (k/6.0)*tmp;
        }
        for(int k=1; k<4; ++k) {
          Eigen::Vector3d tmp(laserCloud->points[i+k].x-laserCloud->points[i].x, laserCloud->points[i+k].y-laserCloud->points[i].y, laserCloud->points[i+k].z-laserCloud->points[i].z);
          if (tmp.norm() > 1e-6) tmp.normalize(); norm_back += (k/6.0)*tmp;
        }
        double cc = 0.0;
        if (norm_front.norm() > 1e-6 && norm_back.norm() > 1e-6) cc = std::abs(norm_front.dot(norm_back) / (norm_front.norm()*norm_back.norm()));
        
        if (cc < 0.8) debugnum3++;
        else CloudFeatureFlag_[i] = 0;
        continue;
      }
    }

    for (int i = 0; i < cloudSize; i++) {
      // The original code for scanRegistration.cpp (not horizon) had a line here to set intensity:
      // laserCloud->points[i].intensity = double(CloudFeatureFlag_[i]) / 10000;
      // This was commented out in my previous refactoring of scanRegistration.cpp.
      // For scanRegistration_horizon.cpp, the intensity field is used for scanID input.
      // So, it should not be overwritten here with feature flags.
      // The PointType is PointXYZINormal, so .normal and .curvature fields are available if needed for output.

      if (CloudFeatureFlag_[i] == 1) {
        surfPointsFlat.push_back(laserCloud->points[i]);
      } else if (CloudFeatureFlag_[i] == 100 || CloudFeatureFlag_[i] == 150) {
        cornerPointsSharp.push_back(laserCloud->points[i]);
      }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Horizon - ALL point: %d, outliers: %d", cloudSize, debugnum1);
    RCLCPP_DEBUG(this->get_logger(), "Horizon - break points: %d, break feature: %d", debugnum2, debugnum3);
    RCLCPP_DEBUG(this->get_logger(), "Horizon - normal points: %d, surf-surf feature: %d", debugnum4, debugnum5);

    // Publish results
    sensor_msgs::msg::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header = laserCloudMsg->header;
    // laserCloudOutMsg.header.frame_id = "/livox"; // ensure this is correct, or use from input msg
    pubLaserCloud_->publish(laserCloudOutMsg);

    sensor_msgs::msg::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header = laserCloudMsg->header;
    pubCornerPointsSharp_->publish(cornerPointsSharpMsg);

    sensor_msgs::msg::PointCloud2 surfPointsFlatMsg;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlatMsg);
    surfPointsFlatMsg.header = laserCloudMsg->header;
    pubSurfPointsFlat_->publish(surfPointsFlatMsg);
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanRegistrationHorizon>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
