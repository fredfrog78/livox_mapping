#ifndef ADAPTIVE_PARAMETER_MANAGER_H
#define ADAPTIVE_PARAMETER_MANAGER_H

#include "adaptive_parameter_manager_types.h"
#include <rclcpp/rclcpp.hpp> // For rclcpp::Node, parameters, logging

// Forward declaration if LaserMapping/ScanRegistration specific types are needed directly
// For now, we'll assume health status comes in as the enums defined in _types.h

namespace loam_adaptive_parameter_manager {

class AdaptiveParameterManager : public rclcpp::Node {
public:
    AdaptiveParameterManager();

    // Method to be called when new health status from ScanRegistration is available
    // CONCEPTUAL INTEGRATION:
    // - scanRegistration node(s) would need to be modified to assess their operational health
    //   (e.g., based on current warning conditions like low feature counts).
    // - This health status (e.g., ScanRegistrationHealth::LOW_SHARP_FEATURES) would be published
    //   on a dedicated ROS 2 topic.
    // - This AdaptiveParameterManager node would subscribe to that topic, and its callback
    //   would invoke this updateScanRegistrationHealth method.
    void updateScanRegistrationHealth(ScanRegistrationHealth sr_health);

    // Method to be called when new health status from LaserMapping is available
    // CONCEPTUAL INTEGRATION:
    // - laserMapping node would need to be modified to assess its operational health
    //   (e.g., based on ICP performance, feature counts post-filtering, warning conditions).
    // - This health status (e.g., LaserMappingHealth::LOW_ICP_CORRESPONDENCES) would be published
    //   on a dedicated ROS 2 topic.
    // - This AdaptiveParameterManager node would subscribe to that topic, and its callback
    //   would invoke this updateLaserMappingHealth method.
    void updateLaserMappingHealth(LaserMappingHealth lm_health);

    // Method to trigger a parameter adjustment check
    // This could be called periodically by a timer or after every health update
    void processHealthAndAdjustParameters();

    // Getters for current adaptive parameters (for external query or logging)
    double getCurrentCornerFilterSize() const { return current_filter_parameter_corner_; }
    double getCurrentSurfFilterSize() const { return current_filter_parameter_surf_; }

private:
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

    // For resource usage simulation / overload detection (Plan step 4)
    int consecutive_icp_issue_warnings_;
    int consecutive_healthy_cycles_; // Added to manage cooldown reset
    static const int ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ = 3;
    static const int HEALTHY_CYCLES_TO_RESET_COOLDOWN_ = 5; // Added
    bool overload_cooldown_active_;

    // Add a timer for cooldown management if necessary, or handle in processHealth
    // rclcpp::TimerBase::SharedPtr cooldown_reset_timer_;
    // void resetCooldownCallback();

    // Helper methods for decision logic
    void initializeParameters();
    void determineSystemHealth();
    // CONCEPTUAL INTEGRATION:
    // This method, in a full implementation, would use a ROS 2 parameter client
    // to dynamically set 'filter_parameter_corner' and 'filter_parameter_surf'
    // on the running laserMapping node. The laserMapping node would need to be
    // set up to allow dynamic parameter updates and use them.
    void applyParameterChanges();

    // ROS 2 specific members (e.g., parameter client if used for actual updates)
    // For now, we'll focus on logic. Actual parameter update is conceptual.
    // std::shared_ptr<rclcpp::SyncParametersClient> laser_mapping_param_client_;
};

} // namespace loam_adaptive_parameter_manager

#endif // ADAPTIVE_PARAMETER_MANAGER_H
