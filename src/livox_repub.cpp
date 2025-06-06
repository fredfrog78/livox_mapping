#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp" // Assuming this is the ROS2 path
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class LivoxRepub : public rclcpp::Node {
public:
  LivoxRepub() : Node("livox_repub_node"), TO_MERGE_CNT(1) {
    // Declare parameter for TO_MERGE_CNT if desired, e.g.
    this->declare_parameter<int>("to_merge_count", 1);
    this->get_parameter("to_merge_count", TO_MERGE_CNT);
    
    RCLCPP_INFO(this->get_logger(), "start livox_repub_node. Merging %ld messages.", TO_MERGE_CNT);

    pub_pcl_out1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_pcl0", rclcpp::QoS(100)); // Keep original queue size
    sub_livox_msg1_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", rclcpp::QoS(100), // Keep original queue size
        std::bind(&LivoxRepub::LivoxMsgCbk1, this, std::placeholders::_1));
  }

private:
  void LivoxMsgCbk1(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in) {
    livox_data_.push_back(livox_msg_in);
    if (livox_data_.size() < static_cast<size_t>(TO_MERGE_CNT)) {
      return;
    }

    pcl::PointCloud<PointType> pcl_in;

    for (size_t j = 0; j < livox_data_.size(); j++) {
      const auto& livox_msg = livox_data_[j];
      // Assuming CustomMsg has a field 'points' which is a vector of point-like structures
      // And each point has x, y, z, offset_time, line, reflectivity
      // And CustomMsg has point_num and timebase fields.
      
      // It's important to check the actual structure of livox_ros_driver2::msg::CustomMsg in ROS2
      // The original code implies livox_msg->points.back().offset_time exists.
      // If points is empty, this would be an error.
      if (livox_msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received Livox message with no points.");
        continue; 
      }
      auto time_end = livox_msg->points.back().offset_time; // Potential issue if points can be empty

      for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
        PointType pt;
        pt.x = livox_msg->points[i].x;
        pt.y = livox_msg->points[i].y;
        pt.z = livox_msg->points[i].z;
        
        float s = 0.0f;
        if (time_end > 0) { // Avoid division by zero
             s = static_cast<float>(livox_msg->points[i].offset_time) / static_cast<float>(time_end);
        }

        // The integer part is line number and the decimal part is reflectivity / 10000.0
        // This encoding might need re-evaluation for clarity or standard PCL fields.
        pt.intensity = static_cast<float>(livox_msg->points[i].line) + 
                       static_cast<float>(livox_msg->points[i].reflectivity) / 10000.0f; 
        pcl_in.push_back(pt);
      }
    }

    if (pcl_in.points.empty() || livox_data_.empty()) {
        livox_data_.clear(); // Clear buffer even if no points were processed
        return; // Don't publish if no points
    }

    uint64_t timebase_ns = livox_data_[0]->timebase;
    rclcpp::Time timestamp(timebase_ns);

    sensor_msgs::msg::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg); // Converts PCL to sensor_msgs::PointCloud2
    
    pcl_ros_msg.header.stamp = timestamp;
    pcl_ros_msg.header.frame_id = "/livox"; // Consider making this a parameter
    
    pub_pcl_out1_->publish(pcl_ros_msg);
    livox_data_.clear();
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out1_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_msg1_;
  
  std::vector<livox_ros_driver2::msg::CustomMsg::SharedPtr> livox_data_;
  long int TO_MERGE_CNT; // Using long int from parameter
  // constexpr static bool b_dbg_line = false; // Not used in this refactoring
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxRepub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
