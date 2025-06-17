/**
 * @file adaptive_parameter_manager.h
 * @brief Defines the AdaptiveParameterManager class for dynamically adjusting LOAM parameters.
 *
 * Purpose:
 * The AdaptiveParameterManager node is designed to improve the robustness and
 * performance of the Livox LOAM mapping pipeline by automatically adjusting
 * key parameters of the laserMapping node in real-time. It reacts to the
 * operational health of the scanRegistration and laserMapping nodes.
 *
 * Key LOAM Parameters Controlled:
 * - 'filter_parameter_corner': Voxel grid leaf size for corner points in laserMapping.
 * - 'filter_parameter_surf': Voxel grid leaf size for surface points in laserMapping.
 *
 * Core Logic Overview:
 * 1. Health Monitoring:
 *    - Subscribes to health status topics from scanRegistration (e.g., /scan_registration/health_status)
 *      and laserMapping (e.g., /laser_mapping/health_status). These topics are expected
 *      to publish integer representations of ScanRegistrationHealth and LaserMappingHealth enums.
 * 2. Health State Smoothing:
 *    - Raw health reports are smoothed by requiring a state to be reported consecutively
 *      (HEALTH_REPORT_STABILITY_THRESHOLD_ times, currently 2) before it's considered "stabilized".
 *    - Decisions are based on these stabilized health states to avoid reacting to transient noise.
 * 3. Decision Making (based on stabilized system health):
 *    - If laserMapping has too few features for ICP: Decrease filter sizes to provide more points.
 *    - If laserMapping ICP is unstable (large corrections, degeneracy):
 *        - If scanRegistration also reports few features: Cautiously decrease filter sizes.
 *        - If scanRegistration is healthy: Increase filter sizes (may help with noisy/dense data).
 *    - If scanRegistration has issues: Maintain current laserMapping filter sizes.
 *    - If system is HEALTHY:
 *        - Cautious Probing: After a period of sustained health (PROBING_AFTER_N_HEALTHY_CYCLES_, currently 2),
 *          slowly increase filter sizes to test for resource savings.
 * 4. Overload Detection & Cooldown:
 *    - If ICP instability persists for several cycles (ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_, currently 3),
 *      an "overload" is detected.
 *    - Cooldown mode is activated: Filter sizes are temporarily increased to help stabilize,
 *      and further reductions are paused.
 *    - Cooldown is reset after a period of sustained health (HEALTHY_CYCLES_TO_RESET_COOLDOWN_, currently 5).
 * 5. Parameter Updates:
 *    - Uses an asynchronous ROS 2 parameter client to set the 'filter_parameter_corner'
 *      and 'filter_parameter_surf' on the target laserMapping node.
 *
 * Assumptions:
 * - scanRegistration and laserMapping nodes have been modified to publish their health
 *   status on the expected topics (e.g., /scan_registration/health_status,
 *   /laser_mapping/health_status) using std_msgs::msg::Int32, where the integer
 *   corresponds to the enums in adaptive_parameter_manager_types.h.
 * - The target laserMapping node (default: "laser_mapping_node", configurable via ROS parameter
 *   "target_node_name" for the AdaptiveParameterManager node) is running and its parameters
 *   'filter_parameter_corner' and 'filter_parameter_surf' are dynamically updatable.
 * - The node operates primarily in a single-threaded execution model for its callbacks,
 *   ensuring safety for shared member variable access without explicit mutexes.
 */
#ifndef ADAPTIVE_PARAMETER_MANAGER_H
#define ADAPTIVE_PARAMETER_MANAGER_H

#include "adaptive_parameter_manager_types.h"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp" // Added for resource/pipeline metrics
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter.hpp>
#include <string>
#include <limits> // For std::numeric_limits
#include "livox_mapping/srv/get_parameter_statistics.hpp" // Adjusted include path

namespace loam_adaptive_parameter_manager {

class AdaptiveParameterManager : public rclcpp::Node {
public:
    AdaptiveParameterManager();

    void processHealthAndAdjustParameters();

    double getCurrentCornerFilterSize() const { return current_filter_parameter_corner_; }
    double getCurrentSurfFilterSize() const { return current_filter_parameter_surf_; }
    int getCurrentICPIterations() const { return current_icp_iterations_; }


    // Public constants
    static const int HEALTH_REPORT_STABILITY_THRESHOLD_ = 2; // Min consecutive reports for a state to be "stabilized"
    static const int ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ = 3;     // System states causing ICP warnings to trigger overload
    static const int HEALTHY_CYCLES_TO_RESET_COOLDOWN_ = 5; // HEALTHY states to reset overload cooldown
    static const int PROBING_AFTER_N_HEALTHY_CYCLES_ = 2;   // HEALTHY states before probing parameters

protected:
    // Internal state variables needed by TestableAdaptiveParameterManager
    ScanRegistrationHealth stabilized_sr_health_;   // Health of ScanRegistration after meeting stability threshold
    LaserMappingHealth stabilized_lm_health_;     // Health of LaserMapping after meeting stability threshold

    float latest_cpu_load_;
    float latest_memory_usage_;
    float latest_pipeline_latency_sec_;
    rclcpp::Time last_cpu_load_timestamp_;
    rclcpp::Time last_memory_usage_timestamp_;
    rclcpp::Time last_pipeline_latency_timestamp_;

    SystemHealth current_system_health_;        // Overall system health, derived from stabilized component healths

    double current_filter_parameter_corner_;
    double current_filter_parameter_surf_;
    int current_icp_iterations_;

    bool overload_cooldown_active_;             // Flag indicating if overload cooldown is active
    int consecutive_healthy_cycles_;          // Counts consecutive HEALTHY system states
    int consecutive_icp_issue_warnings_;      // Counts consecutive LASER_MAPPING_ICP_UNSTABLE states

    // Configuration for adaptable parameters: their defaults, min/max bounds
    double default_filter_parameter_corner_;
    double min_filter_parameter_corner_;
    double max_filter_parameter_corner_;

    double default_filter_parameter_surf_;
    double min_filter_parameter_surf_;
    double max_filter_parameter_surf_;

    double adjustment_step_small_;              // For gentle probing or cautious adjustments
    double adjustment_step_normal_;             // For standard reactions to issues

    // Configuration for ICP iterations
    int default_icp_iterations_;
    int min_icp_iterations_;
    int max_icp_iterations_;
    int icp_iteration_adjustment_step_;

    // Thresholds for resource monitoring (loaded from ROS parameters)
    double cpu_load_threshold_moderate_;
    double cpu_load_threshold_high_;
    double cpu_load_threshold_critical_;
    double memory_usage_threshold_high_;
    double memory_usage_threshold_critical_;
    double pipeline_latency_threshold_high_sec_;
    double pipeline_latency_threshold_critical_sec_;
    double metric_stale_threshold_sec_;

    // Helper methods needed by TestableAdaptiveParameterManager
    void initializeParameters();                // Loads initial parameter values and configurations
    void determineSystemHealth();               // Determines current_system_health_ from stabilized states
    // applyParameterChanges(); // Not moving to protected as per self-correction in prompt

private:
    // Subscription Callbacks to receive health status from other nodes
    void scanRegistrationHealthCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void laserMappingHealthCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void cpuLoadCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void memoryUsageCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void pipelineLatencyCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // Internal methods to update and stabilize health states
    void updateScanRegistrationHealth(ScanRegistrationHealth sr_health);
    void updateLaserMappingHealth(LaserMappingHealth lm_health);

    // Internal state variables not directly needed by current tests via protected access
    ScanRegistrationHealth latest_sr_health_;       // Raw health from ScanRegistration topic
    LaserMappingHealth latest_lm_health_;         // Raw health from LaserMapping topic
    int sr_health_consecutive_count_;           // Counter for consecutive identical reports of latest_sr_health_
    int lm_health_consecutive_count_;           // Counter for consecutive identical reports of latest_lm_health_
    AdaptiveMode current_adaptive_mode_;        // Current adaptive behavior settings (e.g., enabled)

    // Core logic not directly called by tests
    void applyParameterChanges();               // Applies changes to target node via parameter client
    bool isMetricFresh(const rclcpp::Time& metric_timestamp) const; // Checks if a metric is recent enough

    // ROS 2 specific members
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sr_health_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lm_health_sub_;
    rclcpp::TimerBase::SharedPtr processing_timer_;        // Timer to periodically call processHealthAndAdjustParameters
    std::shared_ptr<rclcpp::AsyncParametersClient> laser_mapping_param_client_; // Async client to set parameters on target
    std::string laser_mapping_node_name_;       // Name of the target node (e.g., "laser_mapping_node")

    // Subscribers for resource and pipeline metrics
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cpu_load_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr memory_usage_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pipeline_latency_sub_;

    // Statistics
    struct ParameterStats {
        double sum_filter_corner = 0.0;
        double sum_filter_surf = 0.0;
        long long sum_icp_iterations = 0;
        long long count = 0;

        double min_filter_corner = std::numeric_limits<double>::max();
        double max_filter_corner = std::numeric_limits<double>::lowest();
        double min_filter_surf = std::numeric_limits<double>::max();
        double max_filter_surf = std::numeric_limits<double>::lowest();
        int min_icp_iterations = std::numeric_limits<int>::max();
        int max_icp_iterations = std::numeric_limits<int>::min();

        void reset() {
            sum_filter_corner = 0.0;
            sum_filter_surf = 0.0;
            sum_icp_iterations = 0;
            count = 0;
            min_filter_corner = std::numeric_limits<double>::max();
            max_filter_corner = std::numeric_limits<double>::lowest();
            min_filter_surf = std::numeric_limits<double>::max();
            max_filter_surf = std::numeric_limits<double>::lowest();
            min_icp_iterations = std::numeric_limits<int>::max();
            max_icp_iterations = std::numeric_limits<int>::min();
        }
    };
    ParameterStats healthy_param_stats_;

    // ROS Service Server for statistics
    rclcpp::Service<livox_mapping::srv::GetParameterStatistics>::SharedPtr stats_service_;
    void getStatisticsServiceCallback(
        const std::shared_ptr<livox_mapping::srv::GetParameterStatistics::Request> request,
        std::shared_ptr<livox_mapping::srv::GetParameterStatistics::Response> response);
};

} // namespace loam_adaptive_parameter_manager

#endif // ADAPTIVE_PARAMETER_MANAGER_H
