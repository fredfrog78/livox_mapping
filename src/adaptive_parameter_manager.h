#ifndef ADAPTIVE_PARAMETER_MANAGER_H
#define ADAPTIVE_PARAMETER_MANAGER_H

#include "adaptive_parameter_manager_types.h"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp" // For subscription
// The following are typically included by rclcpp/parameter_client.hpp or rclcpp.hpp
// #include <rcl_interfaces/srv/set_parameters.hpp>
// #include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter_client.hpp> // For rclcpp::SyncParametersClient
#include <rclcpp/parameter.hpp>      // For rclcpp::Parameter

namespace loam_adaptive_parameter_manager {

class AdaptiveParameterManager : public rclcpp::Node {
public:
    AdaptiveParameterManager();

    // Main processing logic, now called by a timer
    void processHealthAndAdjustParameters();

    // Getters for current adaptive parameters (for external query or logging)
    double getCurrentCornerFilterSize() const { return current_filter_parameter_corner_; }
    double getCurrentSurfFilterSize() const { return current_filter_parameter_surf_; }

private:
    // Subscription Callbacks
    void scanRegistrationHealthCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void laserMappingHealthCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // Update methods called by callbacks to update internal health state
    void updateScanRegistrationHealth(ScanRegistrationHealth sr_health);
    void updateLaserMappingHealth(LaserMappingHealth lm_health);

    // Internal state variables
    ScanRegistrationHealth latest_sr_health_;
    LaserMappingHealth latest_lm_health_;
    SystemHealth current_system_health_;
    AdaptiveMode current_adaptive_mode_;

    // Current values of adaptable parameters
    double current_filter_parameter_corner_;
    double current_filter_parameter_surf_;

    // Default, Min, Max values for parameters
    double default_filter_parameter_corner_;
    double min_filter_parameter_corner_;
    double max_filter_parameter_corner_;

    double default_filter_parameter_surf_;
    double min_filter_parameter_surf_;
    double max_filter_parameter_surf_;

    double adjustment_step_small_; // For gentle probing/increase
    double adjustment_step_normal_; // For reaction to issues

    // For resource usage simulation / overload detection
    int consecutive_icp_issue_warnings_;
    int consecutive_healthy_cycles_;
    static const int ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ = 3;
    static const int HEALTHY_CYCLES_TO_RESET_COOLDOWN_ = 5;
    bool overload_cooldown_active_;

    // Helper methods for decision logic
    void initializeParameters();
    void determineSystemHealth();
    void applyParameterChanges(); // This would interact with LaserMapping node

    // ROS 2 specific members
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sr_health_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lm_health_sub_; // Could add one for _horizon variant
    rclcpp::TimerBase::SharedPtr processing_timer_;
    std::shared_ptr<rclcpp::SyncParametersClient> laser_mapping_param_client_;
    std::string laser_mapping_node_name_ = "laser_mapping_node"; // Target node name
};

} // namespace loam_adaptive_parameter_manager

#endif // ADAPTIVE_PARAMETER_MANAGER_H
