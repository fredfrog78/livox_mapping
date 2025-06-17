/**
 * @file adaptive_parameter_manager.cpp
 * @brief Implements the AdaptiveParameterManager class for dynamically adjusting LOAM parameters.
 *
 * Implementation Details:
 *
 * Key Methods:
 * - Constructor: Initializes ROS 2 node, parameters, subscriptions (to health topics),
 *   parameter client (async for laserMapping), and a periodic timer.
 * - Subscription Callbacks (scanRegistrationHealthCallback, laserMappingHealthCallback):
 *   Receive health status messages (std_msgs::msg::Int32) and call internal update methods.
 * - updateScanRegistrationHealth / updateLaserMappingHealth (private):
 *   Implement health state smoothing. A health state is considered "latest" immediately,
 *   but only becomes "stabilized" after being reported consecutively for
 *   HEALTH_REPORT_STABILITY_THRESHOLD_ times (currently 2). This prevents reacting to noise.
 * - determineSystemHealth (private): Consolidates the "stabilized" health states from
 *   scanRegistration and laserMapping into an overall SystemHealth enum, which drives
 *   the main decision logic.
 * - processHealthAndAdjustParameters (timer callback): This is the main logic loop.
 *   It calls determineSystemHealth, then based on SystemHealth, it implements rules
 *   to adjust filter_parameter_corner and filter_parameter_surf. It also handles
 *   overload detection/cooldown and cautious probing logic.
 * - applyParameterChanges (private): Uses the AsyncParametersClient to send updated
 *   parameter values to the target laserMapping node. Handles the future/callback
 *   from the async call to log results.
 *
 * Internal Parameters & Thresholds (Default values set in initializeParameters unless noted):
 * - Target Node for Parameters: 'target_node_name' (ROS param, default: "laser_mapping_node").
 *   The APM attempts to set parameters on this node.
 * - APM Timer Period: 'timer_period_sec' (ROS param, default: 1.0s).
 *   Frequency at which processHealthAndAdjustParameters is called. (Note: current impl. uses hardcoded 1s)
 * - Filter Size Bounds:
 *   - min_filter_parameter_corner_ (default: 0.05), max_filter_parameter_corner_ (default: 0.4)
 *   - min_filter_parameter_surf_ (default: 0.1), max_filter_parameter_surf_ (default: 0.8)
 * - Adjustment Steps:
 *   - adjustment_step_small_ (default: 0.005): For gentle probing or cautious adjustments.
 *   - adjustment_step_normal_ (default: 0.02): For standard reactions to issues.
 * - Health Smoothing:
 *   - HEALTH_REPORT_STABILITY_THRESHOLD_ (static const: 2): Number of consecutive identical
 *     health reports required for a component's health to be "stabilized".
 * - Overload Detection & Cooldown:
 *   - ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ (static const: 3): Consecutive "LASER_MAPPING_ICP_UNSTABLE"
 *     system states needed to trigger overload cooldown.
 *   - HEALTHY_CYCLES_TO_RESET_COOLDOWN_ (static const: 5): Consecutive "HEALTHY" system
 *     states needed to deactivate overload cooldown.
 * - Cautious Probing:
 *   - PROBING_AFTER_N_HEALTHY_CYCLES_ (static const: 2): Number of "HEALTHY" system states
 *     (when not in cooldown) required before APM attempts to increase filter sizes.
 *
 * Callback Safety & Threading:
 * - The node is designed with the assumption of a single-threaded executor (e.g.,
 *   as typically created by rclcpp::spin()). Callbacks (timer, subscription, async parameter result)
 *   are expected to run sequentially on that thread. Access to shared member variables
 *   is therefore not currently protected by mutexes. If a multi-threaded executor
 *   is used, this aspect would need review and potential addition of synchronization primitives.
 */
#include "adaptive_parameter_manager.h"
#include <algorithm> // For std::min/max
#include <chrono>    // For timer duration and service timeout
#include <stdexcept> // For std::runtime_error
#include <cmath>     // For std::abs
#include <sstream>   // For std::ostringstream
#include <iomanip>   // For std::fixed, std::setprecision
#include <limits>    // For std::numeric_limits in callback

namespace loam_adaptive_parameter_manager {

// const double CPU_LOAD_THRESHOLD_HIGH_ = 0.85; // Replaced by ROS parameter
// const double CPU_LOAD_THRESHOLD_CRITICAL_ = 0.95; // Replaced by ROS parameter
// const double MEMORY_USAGE_THRESHOLD_HIGH_ = 0.85; // Replaced by ROS parameter
// const double MEMORY_USAGE_THRESHOLD_CRITICAL_ = 0.95; // Replaced by ROS parameter
// const double PIPELINE_LATENCY_THRESHOLD_HIGH_SEC_ = 0.5; // Replaced by ROS parameter
// const double PIPELINE_LATENCY_THRESHOLD_CRITICAL_SEC_ = 1.0; // Replaced by ROS parameter
// const double METRIC_STALE_THRESHOLD_SEC_ = 5.0; // Replaced by ROS parameter

const char* RCLCPP_SYSTEM_HEALTH_TO_STRING(SystemHealth health) {
    switch (health) {
        case SystemHealth::LASER_MAPPING_ICP_UNSTABLE: return "LASER_MAPPING_ICP_UNSTABLE";
        case SystemHealth::LASER_MAPPING_OVERLOADED: return "LASER_MAPPING_OVERLOADED";
        case SystemHealth::PIPELINE_FALLING_BEHIND: return "PIPELINE_FALLING_BEHIND";
        case SystemHealth::HIGH_CPU_LOAD: return "HIGH_CPU_LOAD";
        case SystemHealth::LOW_MEMORY: return "LOW_MEMORY";
        case SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP: return "LASER_MAPPING_FEW_FEATURES_FOR_ICP";
        case SystemHealth::SCAN_REGISTRATION_ISSUES: return "SCAN_REGISTRATION_ISSUES";
        case SystemHealth::HEALTHY: return "HEALTHY";
        case SystemHealth::UNKNOWN: return "UNKNOWN";
        default: return "UNDEFINED_HEALTH_STATE";
    }
}

AdaptiveParameterManager::AdaptiveParameterManager() : Node("adaptive_parameter_manager_node"),
    latest_sr_health_(ScanRegistrationHealth::HEALTHY),
    latest_lm_health_(LaserMappingHealth::HEALTHY),
    stabilized_sr_health_(ScanRegistrationHealth::HEALTHY),
    stabilized_lm_health_(LaserMappingHealth::HEALTHY),
    sr_health_consecutive_count_(0),
    lm_health_consecutive_count_(0),
    current_system_health_(SystemHealth::UNKNOWN),
    consecutive_icp_issue_warnings_(0),
    consecutive_healthy_cycles_(0),
    overload_cooldown_active_(false),
    latest_cpu_load_(0.0f),
    latest_memory_usage_(0.0f),
    latest_pipeline_latency_sec_(0.0f) {

    initializeParameters();

    // Initialize timestamps for metrics. Using this->now() assumes metrics are fresh on first receipt.
    // Alternatively, use rclcpp::Time(0,0, RCL_ROS_TIME) for a very old timestamp if explicit freshness check
    // before first message is needed.
    last_cpu_load_timestamp_ = this->now();
    last_memory_usage_timestamp_ = this->now();
    last_pipeline_latency_timestamp_ = this->now();

    auto default_qos = rclcpp::SystemDefaultsQoS();
    sr_health_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/scan_registration/health_status",
        default_qos,
        std::bind(&AdaptiveParameterManager::scanRegistrationHealthCallback, this, std::placeholders::_1));

    lm_health_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/laser_mapping/health_status",
        default_qos,
        std::bind(&AdaptiveParameterManager::laserMappingHealthCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "AdaptiveParameterManager node initialized.");
    if(sr_health_sub_) RCLCPP_INFO(this->get_logger(), "Subscribed to %s", sr_health_sub_->get_topic_name());
    if(lm_health_sub_) RCLCPP_INFO(this->get_logger(), "Subscribed to %s", lm_health_sub_->get_topic_name());

    cpu_load_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/system_monitor/cpu_load",
        default_qos,
        std::bind(&AdaptiveParameterManager::cpuLoadCallback, this, std::placeholders::_1));

    memory_usage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/system_monitor/memory_usage",
        default_qos,
        std::bind(&AdaptiveParameterManager::memoryUsageCallback, this, std::placeholders::_1));

    pipeline_latency_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/laser_mapping/pipeline_latency_sec",
        default_qos,
        std::bind(&AdaptiveParameterManager::pipelineLatencyCallback, this, std::placeholders::_1));

    if(cpu_load_sub_) RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cpu_load_sub_->get_topic_name());
    if(memory_usage_sub_) RCLCPP_INFO(this->get_logger(), "Subscribed to %s", memory_usage_sub_->get_topic_name());
    if(pipeline_latency_sub_) RCLCPP_INFO(this->get_logger(), "Subscribed to %s", pipeline_latency_sub_->get_topic_name());

    RCLCPP_INFO(this->get_logger(), "Initial Corner Filter: %.3f, Surf Filter: %.3f", current_filter_parameter_corner_, current_filter_parameter_surf_);

    laser_mapping_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, laser_mapping_node_name_);

    using namespace std::chrono_literals;
    // Timer period is hardcoded at 1s. Can be made configurable if needed by reading param before timer creation.
    processing_timer_ = this->create_wall_timer(
        1s,
        std::bind(&AdaptiveParameterManager::processHealthAndAdjustParameters, this));
    RCLCPP_INFO(this->get_logger(), "Processing timer started (1s period). Will adjust parameters for: %s", laser_mapping_node_name_.c_str());

    healthy_param_stats_.reset();
    RCLCPP_INFO(this->get_logger(), "Parameter statistics initialized.");

    stats_service_ = this->create_service<livox_mapping::srv::GetParameterStatistics>(
        "~/get_parameter_statistics",
        std::bind(&AdaptiveParameterManager::getStatisticsServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "GetParameterStatistics service created at: %s/get_parameter_statistics", this->get_name());
}

bool AdaptiveParameterManager::isMetricFresh(const rclcpp::Time& metric_timestamp) const {
    if (metric_timestamp.seconds() == 0 && metric_timestamp.nanoseconds() == 0) {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Metric timestamp is zero, assuming stale.");
        return false;
    }
    try {
        if (this->get_clock()->now().seconds() == 0 && this->get_clock()->now().nanoseconds() == 0) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Clock not yet valid (returns zero time), cannot check metric freshness accurately. Assuming stale.");
            return false;
        }
        return (this->get_clock()->now() - metric_timestamp).seconds() < metric_stale_threshold_sec_;
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Error calculating time difference for metric freshness: %s. Assuming stale.", e.what());
        return false;
    }
}

void AdaptiveParameterManager::scanRegistrationHealthCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    ScanRegistrationHealth new_health = static_cast<ScanRegistrationHealth>(msg->data);
    updateScanRegistrationHealth(new_health); // Process and stabilize the received health
}

void AdaptiveParameterManager::laserMappingHealthCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    LaserMappingHealth new_health = static_cast<LaserMappingHealth>(msg->data);
    updateLaserMappingHealth(new_health); // Process and stabilize the received health
}

void AdaptiveParameterManager::initializeParameters() {
    // Declare and get parameters for this node's operation
    this->declare_parameter<std::string>("target_node_name", "laser_mapping_node");
    this->get_parameter("target_node_name", laser_mapping_node_name_);

    this->declare_parameter<double>("timer_period_sec", 1.0);
    // double timer_period_s = this->get_parameter("timer_period_sec").as_double();
    // Note: To make timer_period_sec dynamic from param, timer creation in constructor needs adjustment.

    // Initialize default values, min/max bounds, and adjustment steps for controlled parameters
    default_filter_parameter_corner_ = 0.2;
    min_filter_parameter_corner_ = 0.05;
    max_filter_parameter_corner_ = 0.4;
    default_filter_parameter_surf_ = 0.4;
    min_filter_parameter_surf_ = 0.1;
    max_filter_parameter_surf_ = 0.8;
    current_filter_parameter_corner_ = default_filter_parameter_corner_;
    current_filter_parameter_surf_ = default_filter_parameter_surf_;
    adjustment_step_small_ = 0.005;
    adjustment_step_normal_ = 0.02;
    current_adaptive_mode_.enabled = true; // Enable adaptive behavior by default

    // Initialize ICP iteration parameters
    this->declare_parameter<int>("icp_iterations.default", 10);
    this->get_parameter("icp_iterations.default", default_icp_iterations_);
    this->declare_parameter<int>("icp_iterations.min", 5);
    this->get_parameter("icp_iterations.min", min_icp_iterations_);
    this->declare_parameter<int>("icp_iterations.max", 20);
    this->get_parameter("icp_iterations.max", max_icp_iterations_);
    current_icp_iterations_ = default_icp_iterations_;

    this->declare_parameter<int>("icp_iterations.adjustment_step", 1);
    this->get_parameter("icp_iterations.adjustment_step", icp_iteration_adjustment_step_);

    // Thresholds for resource monitoring
    this->declare_parameter<double>("thresholds.cpu_load.moderate", 0.65);
    this->get_parameter("thresholds.cpu_load.moderate", cpu_load_threshold_moderate_);
    this->declare_parameter<double>("thresholds.cpu_load.high", 0.85);
    this->get_parameter("thresholds.cpu_load.high", cpu_load_threshold_high_);
    this->declare_parameter<double>("thresholds.cpu_load.critical", 0.95);
    this->get_parameter("thresholds.cpu_load.critical", cpu_load_threshold_critical_);

    this->declare_parameter<double>("thresholds.memory_usage.high", 0.85);
    this->get_parameter("thresholds.memory_usage.high", memory_usage_threshold_high_);
    this->declare_parameter<double>("thresholds.memory_usage.critical", 0.95);
    this->get_parameter("thresholds.memory_usage.critical", memory_usage_threshold_critical_);

    this->declare_parameter<double>("thresholds.pipeline_latency.high_sec", 0.5);
    this->get_parameter("thresholds.pipeline_latency.high_sec", pipeline_latency_threshold_high_sec_);
    this->declare_parameter<double>("thresholds.pipeline_latency.critical_sec", 1.0);
    this->get_parameter("thresholds.pipeline_latency.critical_sec", pipeline_latency_threshold_critical_sec_);

    this->declare_parameter<double>("thresholds.metric_stale_sec", 5.0);
    this->get_parameter("thresholds.metric_stale_sec", metric_stale_threshold_sec_);

    // Log initial settings
    RCLCPP_INFO(this->get_logger(), "ICP Iterations: Initial %d (Min: %d, Max: %d, Step: %d)",
                current_icp_iterations_, min_icp_iterations_, max_icp_iterations_, icp_iteration_adjustment_step_);
    RCLCPP_INFO(this->get_logger(), "CPU Thresholds: Moderate %.2f, High %.2f, Critical %.2f",
                   cpu_load_threshold_moderate_, cpu_load_threshold_high_, cpu_load_threshold_critical_);
    RCLCPP_INFO(this->get_logger(), "Mem Thresholds: High %.2f, Critical %.2f", memory_usage_threshold_high_, memory_usage_threshold_critical_);
    RCLCPP_INFO(this->get_logger(), "Latency Thresholds: High %.2fs, Critical %.2fs", pipeline_latency_threshold_high_sec_, pipeline_latency_threshold_critical_sec_);
    RCLCPP_INFO(this->get_logger(), "Metric Stale Threshold: %.2fs", metric_stale_threshold_sec_);
}

void AdaptiveParameterManager::updateScanRegistrationHealth(ScanRegistrationHealth sr_health) {
    if (latest_sr_health_ == sr_health) {
        sr_health_consecutive_count_++; // Increment count for the same health state
    } else {
        // Health state changed, reset count; this new state is the first in its sequence.
        latest_sr_health_ = sr_health;
        sr_health_consecutive_count_ = 1;
    }

    // If the health state has been reported consecutively enough times, update the stabilized health
    if (sr_health_consecutive_count_ >= HEALTH_REPORT_STABILITY_THRESHOLD_) {
        if (stabilized_sr_health_ != latest_sr_health_) {
            stabilized_sr_health_ = latest_sr_health_;
            RCLCPP_INFO(this->get_logger(), "ScanRegistration Health Stabilized to: %d after %d reports", static_cast<int>(stabilized_sr_health_), sr_health_consecutive_count_);
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Raw ScanRegistration Health Updated: %d (count: %d, stabilized: %d)",
        static_cast<int>(latest_sr_health_), sr_health_consecutive_count_, static_cast<int>(stabilized_sr_health_));
}

void AdaptiveParameterManager::updateLaserMappingHealth(LaserMappingHealth lm_health) {
    if (latest_lm_health_ == lm_health) {
        lm_health_consecutive_count_++; // Increment count for the same health state
    } else {
        // Health state changed, reset count; this new state is the first in its sequence.
        latest_lm_health_ = lm_health;
        lm_health_consecutive_count_ = 1;
    }

    // If the health state has been reported consecutively enough times, update the stabilized health
    if (lm_health_consecutive_count_ >= HEALTH_REPORT_STABILITY_THRESHOLD_) {
        if (stabilized_lm_health_ != latest_lm_health_) {
            stabilized_lm_health_ = latest_lm_health_;
            RCLCPP_INFO(this->get_logger(), "LaserMapping Health Stabilized to: %d after %d reports", static_cast<int>(stabilized_lm_health_), lm_health_consecutive_count_);
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Raw LaserMapping Health Updated: %d (count: %d, stabilized: %d)",
        static_cast<int>(latest_lm_health_), lm_health_consecutive_count_, static_cast<int>(stabilized_lm_health_));
}

void AdaptiveParameterManager::determineSystemHealth() {
    // Priority: ICP Unstable > Latency (Critical) > CPU (Critical) > Memory (Critical) >
    //           Latency (High) > CPU (High) > Memory (High) > Few Features > SR Issues > Healthy > Unknown

    // 1. Check for critical LaserMapping ICP issues
    if (stabilized_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_ROTATION ||
        stabilized_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_TRANSLATION ||
        stabilized_lm_health_ == LaserMappingHealth::ICP_DEGENERATE) {
        current_system_health_ = SystemHealth::LASER_MAPPING_ICP_UNSTABLE;
        return;
    }

    // 2. Check Critical Pipeline Latency
    if (isMetricFresh(last_pipeline_latency_timestamp_) && latest_pipeline_latency_sec_ > pipeline_latency_threshold_critical_sec_) {
        current_system_health_ = SystemHealth::PIPELINE_FALLING_BEHIND;
        return;
    }

    // 3. Check Critical CPU Load
    if (isMetricFresh(last_cpu_load_timestamp_) && latest_cpu_load_ > cpu_load_threshold_critical_) {
        current_system_health_ = SystemHealth::HIGH_CPU_LOAD;
        return;
    }

    // 4. Check Critical Memory Usage
    if (isMetricFresh(last_memory_usage_timestamp_) && latest_memory_usage_ > memory_usage_threshold_critical_) {
        current_system_health_ = SystemHealth::LOW_MEMORY;
        return;
    }

    // 5. Check High Pipeline Latency (Non-Critical)
    if (isMetricFresh(last_pipeline_latency_timestamp_) && latest_pipeline_latency_sec_ > pipeline_latency_threshold_high_sec_) {
        current_system_health_ = SystemHealth::PIPELINE_FALLING_BEHIND;
        return;
    }

    // 6. Check High CPU Load (Non-Critical)
    if (isMetricFresh(last_cpu_load_timestamp_) && latest_cpu_load_ > cpu_load_threshold_high_) {
        current_system_health_ = SystemHealth::HIGH_CPU_LOAD;
        return;
    }

    // 7. Check High Memory Usage (Non-Critical)
    if (isMetricFresh(last_memory_usage_timestamp_) && latest_memory_usage_ > memory_usage_threshold_high_) {
        current_system_health_ = SystemHealth::LOW_MEMORY;
        return;
    }

    // Stale Metrics Logging - Placed here so critical issues above are caught first.
    // These throttled warnings will appear if metrics are stale but no critical issue was found.
    if (!isMetricFresh(last_pipeline_latency_timestamp_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Pipeline latency metric is stale (last update: %.2fs ago). System health assessment might be based on old data.", (this->get_clock()->now() - last_pipeline_latency_timestamp_).seconds());
    }
    if (!isMetricFresh(last_cpu_load_timestamp_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "CPU load metric is stale (last update: %.2fs ago). System health assessment might be based on old data.", (this->get_clock()->now() - last_cpu_load_timestamp_).seconds());
    }
    if (!isMetricFresh(last_memory_usage_timestamp_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Memory usage metric is stale (last update: %.2fs ago). System health assessment might be based on old data.", (this->get_clock()->now() - last_memory_usage_timestamp_).seconds());
    }

    // 8. Check for issues related to insufficient features for LaserMapping ICP
    // (These are checked after resource issues, as resource constraints might cause these)
    if (stabilized_lm_health_ == LaserMappingHealth::LOW_DOWNSAMPLED_CORNER_FEATURES ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_DOWNSAMPLED_SURF_FEATURES ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_MAP_CORNER_POINTS_ICP ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_MAP_SURF_POINTS_ICP ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_ICP_CORRESPONDENCES) {
        current_system_health_ = SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP;
        return;
    }

    // 9. Check for ScanRegistration issues
    if (stabilized_sr_health_ == ScanRegistrationHealth::LOW_RAW_POINTS ||
        stabilized_sr_health_ == ScanRegistrationHealth::LOW_SHARP_FEATURES ||
        stabilized_sr_health_ == ScanRegistrationHealth::LOW_FLAT_FEATURES) {
        current_system_health_ = SystemHealth::SCAN_REGISTRATION_ISSUES;
        return;
    }

    // 10. If all checks pass, system is Healthy
    // This check must also ensure that metrics are not stale or are within acceptable non-critical ranges
    // to truly be 'HEALTHY'. If metrics are stale, we might not be truly healthy.
    // For this iteration, if metrics are stale, we won't declare HEALTHY, falling through to UNKNOWN.
    bool all_metrics_fresh_or_not_checked =
        (isMetricFresh(last_pipeline_latency_timestamp_) || latest_pipeline_latency_sec_ == 0.0f) && // Consider 0.0f as not yet reported / not an issue
        (isMetricFresh(last_cpu_load_timestamp_) || latest_cpu_load_ == 0.0f) &&
        (isMetricFresh(last_memory_usage_timestamp_) || latest_memory_usage_ == 0.0f);

    if (stabilized_lm_health_ == LaserMappingHealth::HEALTHY &&
        stabilized_sr_health_ == ScanRegistrationHealth::HEALTHY &&
        all_metrics_fresh_or_not_checked) { // Only healthy if component healths are good AND metrics are fresh (or not yet an issue)
        current_system_health_ = SystemHealth::HEALTHY;
        return;
    }

    current_system_health_ = SystemHealth::UNKNOWN; // Default if no other state is determined
}

void AdaptiveParameterManager::processHealthAndAdjustParameters() {
    if (!current_adaptive_mode_.enabled) {
        RCLCPP_DEBUG(this->get_logger(), "Adaptive mode is disabled. No parameter adjustments will be made.");
        return;
    }

    SystemHealth health_before_update = current_system_health_;
    determineSystemHealth();
    if(current_system_health_ != health_before_update) {
        RCLCPP_INFO(this->get_logger(), "System health determined: %s (was %s)",
                    RCLCPP_SYSTEM_HEALTH_TO_STRING(current_system_health_),
                    RCLCPP_SYSTEM_HEALTH_TO_STRING(health_before_update));
    }

    // --- Begin Statistics Collection for HEALTHY state ---
    if (current_system_health_ == SystemHealth::HEALTHY) {
        healthy_param_stats_.sum_filter_corner += current_filter_parameter_corner_;
        healthy_param_stats_.sum_filter_surf += current_filter_parameter_surf_;
        healthy_param_stats_.sum_icp_iterations += current_icp_iterations_;
        healthy_param_stats_.count++;

        if (current_filter_parameter_corner_ < healthy_param_stats_.min_filter_corner) {
            healthy_param_stats_.min_filter_corner = current_filter_parameter_corner_;
        }
        if (current_filter_parameter_corner_ > healthy_param_stats_.max_filter_corner) {
            healthy_param_stats_.max_filter_corner = current_filter_parameter_corner_;
        }

        if (current_filter_parameter_surf_ < healthy_param_stats_.min_filter_surf) {
            healthy_param_stats_.min_filter_surf = current_filter_parameter_surf_;
        }
        if (current_filter_parameter_surf_ > healthy_param_stats_.max_filter_surf) {
            healthy_param_stats_.max_filter_surf = current_filter_parameter_surf_;
        }

        if (current_icp_iterations_ < healthy_param_stats_.min_icp_iterations) {
            healthy_param_stats_.min_icp_iterations = current_icp_iterations_;
        }
        if (current_icp_iterations_ > healthy_param_stats_.max_icp_iterations) {
            healthy_param_stats_.max_icp_iterations = current_icp_iterations_;
        }
    }
    // --- End Statistics Collection ---

    double prev_corner_filter = current_filter_parameter_corner_;
    double prev_surf_filter = current_filter_parameter_surf_;
    int prev_icp_iterations = current_icp_iterations_;

    // Overload Management
    if (current_system_health_ == SystemHealth::LASER_MAPPING_ICP_UNSTABLE) {
        consecutive_icp_issue_warnings_++;
        consecutive_healthy_cycles_ = 0;
        if (consecutive_icp_issue_warnings_ >= ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ && !overload_cooldown_active_) {
            RCLCPP_WARN(this->get_logger(), "Overload detected! Consecutive ICP issues (%d) reached threshold (%d). Activating cooldown.",
                        consecutive_icp_issue_warnings_, ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_);
            overload_cooldown_active_ = true;
            current_icp_iterations_ = std::max(min_icp_iterations_, default_icp_iterations_ - 2); // Keep fixed reduction for now
            current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, default_filter_parameter_corner_ + adjustment_step_normal_ * 2.0);
            current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, default_filter_parameter_surf_ + adjustment_step_normal_ * 2.0);
        }
    } else if (current_system_health_ == SystemHealth::HEALTHY) {
        if (consecutive_icp_issue_warnings_ > 0) {
            RCLCPP_INFO(this->get_logger(), "ICP issues seems resolved (current state: HEALTHY), resetting warning count from %d to 0.", consecutive_icp_issue_warnings_);
            consecutive_icp_issue_warnings_ = 0;
        }
        consecutive_healthy_cycles_++;
        if (overload_cooldown_active_ && consecutive_healthy_cycles_ >= HEALTHY_CYCLES_TO_RESET_COOLDOWN_) {
            RCLCPP_INFO(this->get_logger(), "System stable for %d cycles. Resetting overload cooldown. Parameters will return to default/probing.", consecutive_healthy_cycles_);
            overload_cooldown_active_ = false;
            consecutive_healthy_cycles_ = 0;
            current_icp_iterations_ = default_icp_iterations_;
            current_filter_parameter_corner_ = default_filter_parameter_corner_;
            current_filter_parameter_surf_ = default_filter_parameter_surf_;
        }
    } else {
        consecutive_healthy_cycles_ = 0;
        // Reset ICP warning count if the issue is NOT ICP_UNSTABLE, FEW_FEATURES (which can lead to ICP unstable), or a resource issue (which can also lead to ICP unstable)
        // This prevents warnings from clearing if the system is oscillating between, e.g. HIGH_CPU and ICP_UNSTABLE
        if (current_system_health_ != SystemHealth::LASER_MAPPING_ICP_UNSTABLE &&
            current_system_health_ != SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP &&
            current_system_health_ != SystemHealth::PIPELINE_FALLING_BEHIND &&
            current_system_health_ != SystemHealth::HIGH_CPU_LOAD &&
            current_system_health_ != SystemHealth::LOW_MEMORY) {
           if(consecutive_icp_issue_warnings_ > 0) {
             RCLCPP_INFO(this->get_logger(), "System issue is %s (not directly ICP related), resetting ICP warning count from %d.", RCLCPP_SYSTEM_HEALTH_TO_STRING(current_system_health_), consecutive_icp_issue_warnings_);
             consecutive_icp_issue_warnings_ = 0;
           }
        }
    }

    if (overload_cooldown_active_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Overload cooldown active. Action: Maintaining conservative parameters (ICP: %d, Corner: %.3f, Surf: %.3f).",
            current_icp_iterations_, current_filter_parameter_corner_, current_filter_parameter_surf_);
    } else {
        switch (current_system_health_) {
            case SystemHealth::LASER_MAPPING_ICP_UNSTABLE:
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: LASER_MAPPING_ICP_UNSTABLE (not in cooldown). Action: Decrease ICP iter, Increase filter sizes.");
                current_icp_iterations_ = std::max(min_icp_iterations_, current_icp_iterations_ - icp_iteration_adjustment_step_);
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_normal_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_normal_);
                break;

            case SystemHealth::PIPELINE_FALLING_BEHIND:
            case SystemHealth::HIGH_CPU_LOAD:
            case SystemHealth::LOW_MEMORY:
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: Resource Constraint (%s). Action: Decrease ICP iter primarily, then increase filters if needed.", RCLCPP_SYSTEM_HEALTH_TO_STRING(current_system_health_));
                if (current_icp_iterations_ > min_icp_iterations_) {
                    current_icp_iterations_ = std::max(min_icp_iterations_, current_icp_iterations_ - icp_iteration_adjustment_step_);
                } else if (current_filter_parameter_surf_ < max_filter_parameter_surf_ || current_filter_parameter_corner_ < max_filter_parameter_corner_) {
                    current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_normal_);
                    current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_normal_);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Resource constraint (%s) persists, ICP at min (%d), filters at max (C:%.3f, S:%.3f). No further relaxation possible.",
                                         RCLCPP_SYSTEM_HEALTH_TO_STRING(current_system_health_), min_icp_iterations_, max_filter_parameter_corner_, max_filter_parameter_surf_);
                }
                break;

            case SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP:
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: LASER_MAPPING_FEW_FEATURES_FOR_ICP. Action: Decrease filter sizes.");
                current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_normal_);
                current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_normal_);
                break;

            case SystemHealth::SCAN_REGISTRATION_ISSUES:
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: SCAN_REGISTRATION_ISSUES. Action: Maintain current params. Issue is upstream.");
                break;

            case SystemHealth::HEALTHY:
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"State: HEALTHY. (Consecutive healthy cycles: %d)", consecutive_healthy_cycles_); // Old log
                if (consecutive_healthy_cycles_ >= PROBING_AFTER_N_HEALTHY_CYCLES_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "HEALTHY: Probing. CPU: %.2f (Fresh: %s, Mod Thr: %.2f). ICP: %d (Max: %d), Corner: %.3f (Min: %.3f), Surf: %.3f (Min: %.3f).",
                                 latest_cpu_load_, isMetricFresh(last_cpu_load_timestamp_) ? "yes" : "no", cpu_load_threshold_moderate_,
                                 current_icp_iterations_, max_icp_iterations_,
                                 current_filter_parameter_corner_, min_filter_parameter_corner_,
                                 current_filter_parameter_surf_, min_filter_parameter_surf_);

                    bool cpu_metric_fresh_and_valid = isMetricFresh(last_cpu_load_timestamp_);
                    bool can_increase_icp = current_icp_iterations_ < max_icp_iterations_;
                    bool cpu_ok_for_icp_increase = !cpu_metric_fresh_and_valid || (cpu_metric_fresh_and_valid && latest_cpu_load_ < cpu_load_threshold_moderate_);

                    if (can_increase_icp && cpu_ok_for_icp_increase) {
                        current_icp_iterations_ = std::min(max_icp_iterations_, current_icp_iterations_ + icp_iteration_adjustment_step_);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "HEALTHY: Increasing ICP iterations to %d.", current_icp_iterations_);
                    } else {
                        // Did not increase ICP. Log reason if it wasn't already at max.
                        if (can_increase_icp && !cpu_ok_for_icp_increase) { // Was able to increase, but CPU was too high
                             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                                 "HEALTHY: CPU load (%.2f) is at/above moderate (%.2f). Not increasing ICP iterations from %d.",
                                                 latest_cpu_load_, cpu_load_threshold_moderate_, current_icp_iterations_);
                        } else if (!can_increase_icp) { // Already at max ICP
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                                 "HEALTHY: ICP iterations already at max (%d).", max_icp_iterations_);
                        }

                        // Now, since ICP wasn't increased (or was already maxed), try to reduce filters
                        if (current_filter_parameter_corner_ > min_filter_parameter_corner_ || current_filter_parameter_surf_ > min_filter_parameter_surf_) {
                            if (current_filter_parameter_corner_ > min_filter_parameter_corner_) {
                                 current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_small_);
                            }
                            if (current_filter_parameter_surf_ > min_filter_parameter_surf_) {
                                current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_small_);
                            }
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                                 "HEALTHY: Reducing filter sizes. New Corner: %.3f, New Surf: %.3f",
                                                 current_filter_parameter_corner_, current_filter_parameter_surf_);
                        } else { // ICP is at max (or held due to CPU), and Filters are at min
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                 "HEALTHY: Max accuracy parameters likely set or CPU limiting ICP increase. (ICP: %d, Filters: C=%.3f, S=%.3f, CPU: %.2f)",
                                                 current_icp_iterations_, min_filter_parameter_corner_, min_filter_parameter_surf_, latest_cpu_load_);
                        }
                    }
                } else { // Not enough healthy cycles yet for probing
                     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "HEALTHY: Waiting for %d consecutive cycles before probing (currently %d). Holding parameters.",
                                PROBING_AFTER_N_HEALTHY_CYCLES_, consecutive_healthy_cycles_);
                }
                break;

            case SystemHealth::LASER_MAPPING_OVERLOADED:
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: LASER_MAPPING_OVERLOADED (external). Action: Enforce conservative params.");
                current_icp_iterations_ = std::max(min_icp_iterations_, default_icp_iterations_ - 2 ); // Keep fixed reduction for now
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, default_filter_parameter_corner_ + adjustment_step_normal_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, default_filter_parameter_surf_ + adjustment_step_normal_);
                break;

            case SystemHealth::UNKNOWN:
            default:
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: UNKNOWN or Unhandled (%s). Action: No parameter adjustments.", RCLCPP_SYSTEM_HEALTH_TO_STRING(current_system_health_));
                break;
        }
    }

    // Ensure parameters are always within their defined min/max bounds after any adjustment
    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, std::min(max_filter_parameter_corner_, current_filter_parameter_corner_));
    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, std::min(max_filter_parameter_surf_, current_filter_parameter_surf_));
    current_icp_iterations_ = std::max(min_icp_iterations_, std::min(max_icp_iterations_, current_icp_iterations_));

    if (std::abs(current_filter_parameter_corner_ - prev_corner_filter) > 1e-6 ||
        std::abs(current_filter_parameter_surf_ - prev_surf_filter) > 1e-6 ||
        current_icp_iterations_ != prev_icp_iterations) {
        RCLCPP_INFO(this->get_logger(), "Parameter values changed. New Corner: %.3f (was %.3f), New Surf: %.3f (was %.3f), New ICP Iter: %d (was %d). Applying to %s.",
                    current_filter_parameter_corner_, prev_corner_filter, current_filter_parameter_surf_, prev_surf_filter,
                    current_icp_iterations_, prev_icp_iterations,
                    laser_mapping_node_name_.c_str());
        applyParameterChanges();
    } else {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No parameter changes to apply this cycle. Current state: %s, Corner: %.3f, Surf: %.3f, ICP: %d",
            RCLCPP_SYSTEM_HEALTH_TO_STRING(current_system_health_), current_filter_parameter_corner_, current_filter_parameter_surf_, current_icp_iterations_);
    }
}

void AdaptiveParameterManager::applyParameterChanges() {
    if (!laser_mapping_param_client_) {
        RCLCPP_ERROR(this->get_logger(), "AsyncParameterClient not initialized for '%s'.", laser_mapping_node_name_.c_str());
        return;
    }
    if (!laser_mapping_param_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Parameter service for '%s' is not currently available. Will retry parameter set next cycle.", laser_mapping_node_name_.c_str());
        return;
    }

    std::vector<rclcpp::Parameter> params_to_set;
    params_to_set.emplace_back("filter_parameter_corner", current_filter_parameter_corner_);
    params_to_set.emplace_back("filter_parameter_surf", current_filter_parameter_surf_);
    params_to_set.emplace_back("icp_max_iterations", current_icp_iterations_);

    RCLCPP_DEBUG(this->get_logger(), "Attempting to asynchronously set parameters on '%s': Corner=%.3f, Surf=%.3f, ICP Iterations=%d",
                laser_mapping_node_name_.c_str(), current_filter_parameter_corner_, current_filter_parameter_surf_, current_icp_iterations_);

    auto callback_lambda = [this]
        (std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        std::vector<rcl_interfaces::msg::SetParametersResult> set_results;
        try {
            set_results = future.get(); // This can throw if the service call itself failed (e.g., node disappeared)
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while getting future for set_parameters on '%s': %s",
                         laser_mapping_node_name_.c_str(), e.what());
            // Note: Parameter changes are not reverted here, as the actual state on target is unknown.
            // The system will re-evaluate and attempt to set again if needed in next cycle.
            return;
        }
        bool all_successful = true;
        std::vector<std::string> attempted_param_names = {"filter_parameter_corner", "filter_parameter_surf", "icp_max_iterations"};
        for (size_t i = 0; i < set_results.size(); ++i) {
            if (!set_results[i].successful) {
                std::string param_name = (i < attempted_param_names.size()) ? attempted_param_names[i] : "unknown_param";
                RCLCPP_ERROR(this->get_logger(), "Async set_parameters failed for '%s' on node '%s': (Reason: %s)",
                             param_name.c_str(), laser_mapping_node_name_.c_str(), set_results[i].reason.c_str());
                all_successful = false;
            }
        }
        if (all_successful) {
            RCLCPP_INFO(this->get_logger(), "Async set_parameters call completed successfully for '%s'.", laser_mapping_node_name_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "One or more async parameters failed to apply on '%s'.", laser_mapping_node_name_.c_str());
            // If parameters failed to set, the local current_... values might be out of sync with the target node.
            // The next cycle of processHealthAndAdjustParameters will re-evaluate and attempt to set them again if they are still different from its desired state.
        }
    };

    laser_mapping_param_client_->set_parameters(params_to_set, callback_lambda);
    RCLCPP_DEBUG(this->get_logger(), "Asynchronous parameter set request sent to '%s'. Result will be logged upon completion.", laser_mapping_node_name_.c_str());
}

void AdaptiveParameterManager::cpuLoadCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    latest_cpu_load_ = msg->data;
    last_cpu_load_timestamp_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "Received CPU load: %.2f", latest_cpu_load_);
}

void AdaptiveParameterManager::memoryUsageCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    latest_memory_usage_ = msg->data;
    last_memory_usage_timestamp_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "Received Memory usage: %.2f", latest_memory_usage_);
}

void AdaptiveParameterManager::pipelineLatencyCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    latest_pipeline_latency_sec_ = msg->data;
    last_pipeline_latency_timestamp_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "Received Pipeline Latency: %.3f s", latest_pipeline_latency_sec_);
}

void AdaptiveParameterManager::getStatisticsServiceCallback(
    const std::shared_ptr<livox_mapping::srv::GetParameterStatistics::Request> request,
    std::shared_ptr<livox_mapping::srv::GetParameterStatistics::Response> response) {

    (void)request; // Suppress unused parameter warning if request is empty

    std::ostringstream summary_ss;
    summary_ss << std::fixed << std::setprecision(3); // Set precision for floating point numbers

    summary_ss << "AdaptiveParameterManager Statistics (during HEALTHY state):\n";
    summary_ss << "----------------------------------------------------------\n";
    summary_ss << "Total HEALTHY cycles recorded: " << healthy_param_stats_.count << "\n\n";

    if (healthy_param_stats_.count > 0) {
        // Corner Filter Statistics
        summary_ss << "Corner Filter Size:\n";
        summary_ss << "  Average: " << (healthy_param_stats_.sum_filter_corner / healthy_param_stats_.count) << "\n";
        summary_ss << "  Min    : " << healthy_param_stats_.min_filter_corner << "\n";
        summary_ss << "  Max    : " << healthy_param_stats_.max_filter_corner << "\n\n";

        // Surf Filter Statistics
        summary_ss << "Surf Filter Size:\n";
        summary_ss << "  Average: " << (healthy_param_stats_.sum_filter_surf / healthy_param_stats_.count) << "\n";
        summary_ss << "  Min    : " << healthy_param_stats_.min_filter_surf << "\n";
        summary_ss << "  Max    : " << healthy_param_stats_.max_filter_surf << "\n\n";

        // ICP Iterations Statistics
        summary_ss << "ICP Iterations:\n";
        summary_ss << "  Average: " << static_cast<double>(healthy_param_stats_.sum_icp_iterations) / healthy_param_stats_.count << "\n"; // Cast for double division
        if (healthy_param_stats_.min_icp_iterations == std::numeric_limits<int>::max()) {
             summary_ss << "  Min    : N/A (no data)\n";
        } else {
             summary_ss << "  Min    : " << healthy_param_stats_.min_icp_iterations << "\n";
        }
        if (healthy_param_stats_.max_icp_iterations == std::numeric_limits<int>::min()) {
             summary_ss << "  Max    : N/A (no data)\n";
        } else {
             summary_ss << "  Max    : " << healthy_param_stats_.max_icp_iterations << "\n";
        }
    } else {
        summary_ss << "No HEALTHY cycles recorded yet to calculate parameter statistics.\n";
    }
    summary_ss << "----------------------------------------------------------\n";

    response->statistics_summary = summary_ss.str();
    RCLCPP_INFO(this->get_logger(), "GetParameterStatistics service called. Responding with summary.");
}

} // namespace loam_adaptive_parameter_manager
