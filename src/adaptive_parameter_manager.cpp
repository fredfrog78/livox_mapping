#include "adaptive_parameter_manager.h"
#include <algorithm> // For std::min/max
#include <chrono>    // For timer duration and service timeout

namespace loam_adaptive_parameter_manager {

AdaptiveParameterManager::AdaptiveParameterManager() : Node("adaptive_parameter_manager_node"),
    latest_sr_health_(ScanRegistrationHealth::HEALTHY),
    latest_lm_health_(LaserMappingHealth::HEALTHY),
    current_system_health_(SystemHealth::UNKNOWN),
    consecutive_icp_issue_warnings_(0),
    consecutive_healthy_cycles_(0),
    overload_cooldown_active_(false) {

    initializeParameters(); // Initializes filter params, steps, etc.

    // Initialize Subscribers
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

    // Initialize Asynchronous Parameter Client for laser_mapping_node
    laser_mapping_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, laser_mapping_node_name_);

    // Initialize Timer for periodic processing
    using namespace std::chrono_literals;
    processing_timer_ = this->create_wall_timer(
        1s,
        std::bind(&AdaptiveParameterManager::processHealthAndAdjustParameters, this));
    RCLCPP_INFO(this->get_logger(), "Processing timer started (1s period). Will attempt to adjust parameters for node: %s", laser_mapping_node_name_.c_str());
}

void AdaptiveParameterManager::scanRegistrationHealthCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    ScanRegistrationHealth new_health = static_cast<ScanRegistrationHealth>(msg->data);
    updateScanRegistrationHealth(new_health);
}

void AdaptiveParameterManager::laserMappingHealthCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    LaserMappingHealth new_health = static_cast<LaserMappingHealth>(msg->data);
    updateLaserMappingHealth(new_health);
}

void AdaptiveParameterManager::initializeParameters() {
    this->declare_parameter<std::string>("target_node_name", "laser_mapping_node");
    this->get_parameter("target_node_name", laser_mapping_node_name_);

    // Note: Timer period is hardcoded in constructor for this version.
    // Making it dynamic would require stopping and recreating the timer or using a different timer type.
    this->declare_parameter<double>("timer_period_sec", 1.0);
    // double timer_period_sec = 1.0;
    // this->get_parameter("timer_period_sec", timer_period_sec);


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

    current_adaptive_mode_.enabled = true;
}

void AdaptiveParameterManager::updateScanRegistrationHealth(ScanRegistrationHealth sr_health) {
    if (latest_sr_health_ != sr_health) {
        latest_sr_health_ = sr_health;
        RCLCPP_INFO(this->get_logger(), "ScanRegistration Health Updated: %d", static_cast<int>(sr_health));
    }
}

void AdaptiveParameterManager::updateLaserMappingHealth(LaserMappingHealth lm_health) {
    if (latest_lm_health_ != lm_health) {
        latest_lm_health_ = lm_health;
        RCLCPP_INFO(this->get_logger(), "LaserMapping Health Updated: %d", static_cast<int>(lm_health));
    }
}

void AdaptiveParameterManager::determineSystemHealth() {
    if (latest_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_ROTATION ||
        latest_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_TRANSLATION ||
        latest_lm_health_ == LaserMappingHealth::ICP_DEGENERATE ||
        latest_lm_health_ == LaserMappingHealth::OVERLOADED_POST_ADJUSTMENT) { // OVERLOADED_POST_ADJUSTMENT might be redundant if APM handles overload state internally
        current_system_health_ = SystemHealth::LASER_MAPPING_ICP_UNSTABLE;
        return;
    }
    if (latest_lm_health_ == LaserMappingHealth::LOW_DOWNSAMPLED_CORNER_FEATURES ||
        latest_lm_health_ == LaserMappingHealth::LOW_DOWNSAMPLED_SURF_FEATURES ||
        latest_lm_health_ == LaserMappingHealth::LOW_MAP_CORNER_POINTS_ICP ||
        latest_lm_health_ == LaserMappingHealth::LOW_MAP_SURF_POINTS_ICP ||
        latest_lm_health_ == LaserMappingHealth::LOW_ICP_CORRESPONDENCES) {
        current_system_health_ = SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP;
        return;
    }
    if (latest_sr_health_ == ScanRegistrationHealth::LOW_RAW_POINTS ||
        latest_sr_health_ == ScanRegistrationHealth::LOW_SHARP_FEATURES ||
        latest_sr_health_ == ScanRegistrationHealth::LOW_FLAT_FEATURES) {
        current_system_health_ = SystemHealth::SCAN_REGISTRATION_ISSUES;
        return;
    }
    if (latest_lm_health_ == LaserMappingHealth::HEALTHY && latest_sr_health_ == ScanRegistrationHealth::HEALTHY) {
        current_system_health_ = SystemHealth::HEALTHY;
        return;
    }
    current_system_health_ = SystemHealth::UNKNOWN;
}

void AdaptiveParameterManager::processHealthAndAdjustParameters() {
    if (!current_adaptive_mode_.enabled) {
        RCLCPP_DEBUG(this->get_logger(), "Adaptive mode is disabled. No parameter adjustments will be made.");
        return;
    }

    determineSystemHealth();

    double prev_corner_filter = current_filter_parameter_corner_;
    double prev_surf_filter = current_filter_parameter_surf_;

    if (current_system_health_ == SystemHealth::LASER_MAPPING_ICP_UNSTABLE) {
        consecutive_icp_issue_warnings_++;
        consecutive_healthy_cycles_ = 0;
        if (consecutive_icp_issue_warnings_ >= ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ && !overload_cooldown_active_) {
            RCLCPP_WARN(this->get_logger(), "Overload detected! Activating cooldown. Forcing larger filter sizes.");
            overload_cooldown_active_ = true;
            current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_normal_ * 2.0);
            current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_normal_ * 2.0);
        }
    } else if (current_system_health_ == SystemHealth::HEALTHY) {
        consecutive_icp_issue_warnings_ = 0;
        if (overload_cooldown_active_) {
            consecutive_healthy_cycles_++;
            if (consecutive_healthy_cycles_ >= HEALTHY_CYCLES_TO_RESET_COOLDOWN_) {
                RCLCPP_INFO(this->get_logger(), "System stable. Resetting overload cooldown.");
                overload_cooldown_active_ = false;
                consecutive_healthy_cycles_ = 0;
            }
        }
    } else {
        if (current_system_health_ != SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP) {
           consecutive_icp_issue_warnings_ = 0;
        }
        consecutive_healthy_cycles_ = 0;
    }

    switch (current_system_health_) {
        case SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP:
            RCLCPP_INFO(this->get_logger(), "State: LASER_MAPPING_FEW_FEATURES_FOR_ICP. Adjusting for more features.");
            if (!overload_cooldown_active_) {
                current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_normal_);
                current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_normal_);
            } else { RCLCPP_INFO(this->get_logger(), "Overload cooldown active, no reduction for FEW_FEATURES_ICP."); }
            break;
        case SystemHealth::LASER_MAPPING_ICP_UNSTABLE:
            RCLCPP_INFO(this->get_logger(), "State: LASER_MAPPING_ICP_UNSTABLE. (%d consecutive warnings)", consecutive_icp_issue_warnings_);
            if (overload_cooldown_active_) {
                RCLCPP_INFO(this->get_logger(), "...Overload cooldown active. Attempting to stabilize by increasing filters.");
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
            } else {
                if (latest_sr_health_ != ScanRegistrationHealth::HEALTHY) { // Check if SR is also struggling
                    RCLCPP_INFO(this->get_logger(), "...SR unhealthy. Cautiously decreasing filters.");
                    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_small_);
                    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_small_);
                } else { // SR is healthy, LM instability might be too many points
                    RCLCPP_INFO(this->get_logger(), "...SR healthy. Increasing filters to stabilize ICP.");
                    current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                    current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
                }
            }
            break;
        case SystemHealth::SCAN_REGISTRATION_ISSUES:
            RCLCPP_INFO(this->get_logger(), "State: SCAN_REGISTRATION_ISSUES. Maintaining LM filters.");
            break;
        case SystemHealth::HEALTHY:
            RCLCPP_INFO(this->get_logger(), "State: HEALTHY. (%d consecutive healthy cycles)", consecutive_healthy_cycles_);
            if (!overload_cooldown_active_ && consecutive_healthy_cycles_ == 0) {
                RCLCPP_INFO(this->get_logger(), "Probing for resource savings by increasing filter sizes.");
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
            } else if (overload_cooldown_active_) {
                RCLCPP_INFO(this->get_logger(), "Healthy, but overload cooldown active (%d/%d). Holding params.", consecutive_healthy_cycles_, HEALTHY_CYCLES_TO_RESET_COOLDOWN_);
            }
            break;
        case SystemHealth::UNKNOWN:
        default:
            RCLCPP_INFO(this->get_logger(), "State: UNKNOWN. No parameter adjustments.");
            break;
    }

    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, std::min(max_filter_parameter_corner_, current_filter_parameter_corner_));
    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, std::min(max_filter_parameter_surf_, current_filter_parameter_surf_));

    if (std::abs(current_filter_parameter_corner_ - prev_corner_filter) > 1e-5 ||
        std::abs(current_filter_parameter_surf_ - prev_surf_filter) > 1e-5) {
        RCLCPP_INFO(this->get_logger(), "Parameter values changed. Attempting to apply to %s.", laser_mapping_node_name_.c_str());
        applyParameterChanges();
    } else {
        RCLCPP_DEBUG(this->get_logger(), "No parameter changes to apply this cycle.");
    }
}

void AdaptiveParameterManager::applyParameterChanges() {
    // With AsyncParametersClient, we don't typically wait_for_service here
    // as the call itself is non-blocking. Service availability checks can be done
    // periodically or upon specific failures if needed.
    if (!laser_mapping_param_client_) {
        RCLCPP_ERROR(this->get_logger(), "AsyncParameterClient not initialized for '%s'.", laser_mapping_node_name_.c_str());
        return;
    }

    // Check if the service is actually available (optional, but good practice)
    // This is a non-blocking check.
    if (!laser_mapping_param_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Parameter service for '%s' is not currently available. Will retry later.", laser_mapping_node_name_.c_str());
        return; // Skip this attempt
    }

    std::vector<rclcpp::Parameter> params_to_set;
    params_to_set.emplace_back("filter_parameter_corner", current_filter_parameter_corner_);
    params_to_set.emplace_back("filter_parameter_surf", current_filter_parameter_surf_);

    RCLCPP_INFO(this->get_logger(), "Attempting to asynchronously set parameters on '%s': Corner=%.3f, Surf=%.3f",
                laser_mapping_node_name_.c_str(), current_filter_parameter_corner_, current_filter_parameter_surf_);

    // Define the callback lambda to handle the result of the async operation
    auto callback_lambda = [this, prev_corner = current_filter_parameter_corner_, prev_surf = current_filter_parameter_surf_] // Capture desired previous values
        (std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {

        std::vector<rcl_interfaces::msg::SetParametersResult> set_results;
        try {
            set_results = future.get();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while getting future for set_parameters on '%s': %s",
                         laser_mapping_node_name_.c_str(), e.what());
            // Potentially revert local parameters if an exception means they likely didn't apply
            // current_filter_parameter_corner_ = prev_corner; // Example: Revert on exception
            // current_filter_parameter_surf_ = prev_surf;
            // RCLCPP_INFO(this->get_logger(), "Reverted local parameters due to exception during async set for '%s'.", laser_mapping_node_name_.c_str());
            return;
        }

        bool all_successful = true;
        for (size_t i = 0; i < set_results.size(); ++i) {
            if (!set_results[i].successful) {
                // It's good to know which parameter failed if possible.
                // The result vector corresponds to the order of parameters in the request.
                std::string param_name = (i < 2) ? (i==0 ? "filter_parameter_corner" : "filter_parameter_surf") : "unknown_param";
                RCLCPP_ERROR(this->get_logger(), "Async set_parameters failed for '%s' on node '%s': (Reason: %s)",
                             param_name.c_str(),
                             laser_mapping_node_name_.c_str(),
                             set_results[i].reason.c_str());
                all_successful = false;
            }
        }

        if (all_successful) {
            RCLCPP_INFO(this->get_logger(), "Async set_parameters call completed successfully for '%s'.", laser_mapping_node_name_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "One or more async parameters failed to apply on '%s'.", laser_mapping_node_name_.c_str());
            // Revert local parameters to their state before this failed attempt
            // This is important because the local state should reflect the actual state on the target node.
            // current_filter_parameter_corner_ = prev_corner; // Revert if any parameter in the batch fails
            // current_filter_parameter_surf_ = prev_surf;
            // RCLCPP_INFO(this->get_logger(), "Reverted local parameters due to failed async set for '%s'.", laser_mapping_node_name_.c_str());
            // Note: A more sophisticated retry or error handling mechanism might be needed for production.
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
