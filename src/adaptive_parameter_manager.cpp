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

namespace loam_adaptive_parameter_manager {

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
    overload_cooldown_active_(false) {

    initializeParameters();

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
    RCLCPP_INFO(this->get_logger(), "Initial Corner Filter: %.3f, Surf Filter: %.3f", current_filter_parameter_corner_, current_filter_parameter_surf_);

    laser_mapping_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, laser_mapping_node_name_);

    using namespace std::chrono_literals;
    // Timer period is hardcoded at 1s. Can be made configurable if needed by reading param before timer creation.
    processing_timer_ = this->create_wall_timer(
        1s,
        std::bind(&AdaptiveParameterManager::processHealthAndAdjustParameters, this));
    RCLCPP_INFO(this->get_logger(), "Processing timer started (1s period). Will adjust parameters for: %s", laser_mapping_node_name_.c_str());
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
    // Consolidate stabilized component health into an overall system health diagnosis
    // Priority of issues: ICP Unstable > Few Features for ICP > Scan Registration Issues > Healthy > Unknown

    // Check for critical LaserMapping ICP issues
    if (stabilized_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_ROTATION ||
        stabilized_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_TRANSLATION ||
        stabilized_lm_health_ == LaserMappingHealth::ICP_DEGENERATE) {
        current_system_health_ = SystemHealth::LASER_MAPPING_ICP_UNSTABLE;
        return;
    }
    // Check for issues related to insufficient features for LaserMapping ICP
    if (stabilized_lm_health_ == LaserMappingHealth::LOW_DOWNSAMPLED_CORNER_FEATURES ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_DOWNSAMPLED_SURF_FEATURES ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_MAP_CORNER_POINTS_ICP ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_MAP_SURF_POINTS_ICP ||
        stabilized_lm_health_ == LaserMappingHealth::LOW_ICP_CORRESPONDENCES) {
        current_system_health_ = SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP;
        return;
    }
    // Check for ScanRegistration issues if LaserMapping seems okay regarding features for ICP
    if (stabilized_sr_health_ == ScanRegistrationHealth::LOW_RAW_POINTS ||
        stabilized_sr_health_ == ScanRegistrationHealth::LOW_SHARP_FEATURES ||
        stabilized_sr_health_ == ScanRegistrationHealth::LOW_FLAT_FEATURES) {
        current_system_health_ = SystemHealth::SCAN_REGISTRATION_ISSUES;
        return;
    }
    // If all components report healthy
    if (stabilized_lm_health_ == LaserMappingHealth::HEALTHY && stabilized_sr_health_ == ScanRegistrationHealth::HEALTHY) {
        current_system_health_ = SystemHealth::HEALTHY;
        return;
    }
    // Default to UNKNOWN if no specific condition is met (e.g., one component UNKNOWN, other HEALTHY)
    current_system_health_ = SystemHealth::UNKNOWN;
}

void AdaptiveParameterManager::processHealthAndAdjustParameters() {
    if (!current_adaptive_mode_.enabled) {
        RCLCPP_DEBUG(this->get_logger(), "Adaptive mode is disabled. No parameter adjustments will be made.");
        return;
    }

    determineSystemHealth(); // Update current_system_health_ based on stabilized component health

    double prev_corner_filter = current_filter_parameter_corner_;
    double prev_surf_filter = current_filter_parameter_surf_;

    // Overload detection and healthy cycle counting logic
    if (current_system_health_ == SystemHealth::LASER_MAPPING_ICP_UNSTABLE) {
        consecutive_icp_issue_warnings_++;
        consecutive_healthy_cycles_ = 0;
        if (consecutive_icp_issue_warnings_ >= ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ && !overload_cooldown_active_) {
            RCLCPP_WARN(this->get_logger(), "Overload detected! Consecutive ICP issues (%d) reached threshold (%d). Activating cooldown.",
                        consecutive_icp_issue_warnings_, ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_);
            overload_cooldown_active_ = true;
            // Force a more conservative stance during overload
            current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_normal_ * 2.0);
            current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_normal_ * 2.0);
        }
    } else if (current_system_health_ == SystemHealth::HEALTHY) {
        consecutive_icp_issue_warnings_ = 0;
        consecutive_healthy_cycles_++;
        if (overload_cooldown_active_ && consecutive_healthy_cycles_ >= HEALTHY_CYCLES_TO_RESET_COOLDOWN_) {
            RCLCPP_INFO(this->get_logger(), "System stable for %d cycles. Resetting overload cooldown.", consecutive_healthy_cycles_);
            overload_cooldown_active_ = false;
            consecutive_healthy_cycles_ = 0; // Reset for fresh count towards probing after cooldown
        }
    } else { // For any other non-healthy, non-ICP-unstable state
        consecutive_healthy_cycles_ = 0;
        if (current_system_health_ != SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP) {
           consecutive_icp_issue_warnings_ = 0; // Reset ICP warnings if the issue is not related to few features that might lead to ICP problems
        }
    }

    // Parameter adjustment logic based on current_system_health_
    switch (current_system_health_) {
        case SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP:
            RCLCPP_INFO(this->get_logger(), "State: LASER_MAPPING_FEW_FEATURES_FOR_ICP. Action: Decrease filter sizes.");
            if (!overload_cooldown_active_) {
                current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_normal_);
                current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_normal_);
            } else { RCLCPP_INFO(this->get_logger(), "Overload cooldown active, no reduction for FEW_FEATURES_ICP."); }
            break;
        case SystemHealth::LASER_MAPPING_ICP_UNSTABLE:
            RCLCPP_INFO(this->get_logger(), "State: LASER_MAPPING_ICP_UNSTABLE. (Consecutive warnings: %d)", consecutive_icp_issue_warnings_);
            if (overload_cooldown_active_) {
                RCLCPP_INFO(this->get_logger(), "Action: Overload cooldown active. Maintain or slightly increase filters.");
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
            } else {
                if (stabilized_sr_health_ != ScanRegistrationHealth::HEALTHY) {
                    RCLCPP_INFO(this->get_logger(), "Action: SR unhealthy. Cautiously decrease filters.");
                    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_small_);
                    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_small_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Action: SR healthy. Increase filters to stabilize ICP.");
                    current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                    current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
                }
            }
            break;
        case SystemHealth::SCAN_REGISTRATION_ISSUES:
            RCLCPP_INFO(this->get_logger(), "State: SCAN_REGISTRATION_ISSUES. Action: Maintain LM filters.");
            // No changes to LM filter sizes; SR issues are upstream.
            break;
        case SystemHealth::HEALTHY:
            RCLCPP_INFO(this->get_logger(), "State: HEALTHY. (Consecutive healthy cycles: %d, Cooldown: %s)",
                        consecutive_healthy_cycles_, overload_cooldown_active_ ? "active" : "inactive");
            if (!overload_cooldown_active_ && consecutive_healthy_cycles_ >= PROBING_AFTER_N_HEALTHY_CYCLES_) {
                RCLCPP_INFO(this->get_logger(), "Action: Probing for resource savings (healthy for %d cycles). Increase filter sizes.", consecutive_healthy_cycles_);
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
            } else if (overload_cooldown_active_) {
                RCLCPP_INFO(this->get_logger(), "Action: Healthy, but overload cooldown active. Holding parameters.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Action: Healthy, but waiting for %d consecutive cycles before probing (currently %d). Holding parameters.",
                            PROBING_AFTER_N_HEALTHY_CYCLES_, consecutive_healthy_cycles_);
            }
            break;
        case SystemHealth::UNKNOWN:
        default:
            RCLCPP_INFO(this->get_logger(), "State: UNKNOWN. Action: No parameter adjustments.");
            break;
    }

    // Ensure parameters are always within their defined min/max bounds after any adjustment
    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, std::min(max_filter_parameter_corner_, current_filter_parameter_corner_));
    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, std::min(max_filter_parameter_surf_, current_filter_parameter_surf_));

    // If parameters changed, apply them to the target node
    if (std::abs(current_filter_parameter_corner_ - prev_corner_filter) > 1e-5 ||
        std::abs(current_filter_parameter_surf_ - prev_surf_filter) > 1e-5) {
        RCLCPP_INFO(this->get_logger(), "Parameter values changed. New Corner: %.3f (was %.3f), New Surf: %.3f (was %.3f). Applying to %s.",
                    current_filter_parameter_corner_, prev_corner_filter, current_filter_parameter_surf_, prev_surf_filter,
                    laser_mapping_node_name_.c_str());
        applyParameterChanges();
    } else {
        RCLCPP_DEBUG(this->get_logger(), "No parameter changes to apply this cycle.");
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

    RCLCPP_DEBUG(this->get_logger(), "Attempting to asynchronously set parameters on '%s': Corner=%.3f, Surf=%.3f",
                laser_mapping_node_name_.c_str(), current_filter_parameter_corner_, current_filter_parameter_surf_);

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
        std::vector<std::string> attempted_param_names = {"filter_parameter_corner", "filter_parameter_surf"};
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

} // namespace loam_adaptive_parameter_manager

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto manager_node = std::make_shared<loam_adaptive_parameter_manager::AdaptiveParameterManager>();
    rclcpp::spin(manager_node);
    rclcpp::shutdown();
    return 0;
}
