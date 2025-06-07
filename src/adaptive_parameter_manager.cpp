#include "adaptive_parameter_manager.h"
#include <algorithm> // For std::min/max

namespace loam_adaptive_parameter_manager {

AdaptiveParameterManager::AdaptiveParameterManager() : Node("adaptive_parameter_manager_node"),
    latest_sr_health_(ScanRegistrationHealth::HEALTHY),
    latest_lm_health_(LaserMappingHealth::HEALTHY),
    current_system_health_(SystemHealth::UNKNOWN),
    consecutive_icp_issue_warnings_(0),
    overload_cooldown_active_(false),
    consecutive_healthy_cycles_(0) { // Initialize consecutive_healthy_cycles_

    initializeParameters();
    RCLCPP_INFO(this->get_logger(), "AdaptiveParameterManager node initialized.");
    RCLCPP_INFO(this->get_logger(), "Initial Corner Filter: %.3f, Surf Filter: %.3f", current_filter_parameter_corner_, current_filter_parameter_surf_);
}

void AdaptiveParameterManager::initializeParameters() {
    // Declare parameters for this node (e.g., default, min, max filter sizes)
    // These would typically be read from a config file or launch parameters.
    // For this task, we'll hardcode them for simplicity.

    default_filter_parameter_corner_ = 0.2; // Default from laserMapping
    min_filter_parameter_corner_ = 0.05;   // Example minimum
    max_filter_parameter_corner_ = 0.4;    // Example maximum

    default_filter_parameter_surf_ = 0.4;  // Default from laserMapping
    min_filter_parameter_surf_ = 0.1;     // Example minimum
    max_filter_parameter_surf_ = 0.8;      // Example maximum

    current_filter_parameter_corner_ = default_filter_parameter_corner_;
    current_filter_parameter_surf_ = default_filter_parameter_surf_;

    adjustment_step_small_ = 0.005; // For gentle probing
    adjustment_step_normal_ = 0.02; // For reacting to issues

    current_adaptive_mode_.enabled = true; // Enable by default

    // Placeholder: In a real system, you might have a parameter client to get initial values from laserMapping
    // Or laserMapping could publish its current operational values.
}

void AdaptiveParameterManager::updateScanRegistrationHealth(ScanRegistrationHealth sr_health) {
    if (latest_sr_health_ != sr_health) {
        latest_sr_health_ = sr_health;
        RCLCPP_DEBUG(this->get_logger(), "ScanRegistration Health Updated: %d", static_cast<int>(sr_health));
        // processHealthAndAdjustParameters(); // Optionally call immediately
    }
}

void AdaptiveParameterManager::updateLaserMappingHealth(LaserMappingHealth lm_health) {
    if (latest_lm_health_ != lm_health) {
        latest_lm_health_ = lm_health;
        RCLCPP_DEBUG(this->get_logger(), "LaserMapping Health Updated: %d", static_cast<int>(lm_health));
        // processHealthAndAdjustParameters(); // Optionally call immediately
    }
}

void AdaptiveParameterManager::determineSystemHealth() {
    // Consolidate SR and LM health into a single SystemHealth
    // This is a simplified decision tree. More complex logic could exist.

    // Order of checks matters: Unstable ICP is often a more critical issue.
    if (latest_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_ROTATION ||
        latest_lm_health_ == LaserMappingHealth::HIGH_ICP_DELTA_TRANSLATION ||
        latest_lm_health_ == LaserMappingHealth::ICP_DEGENERATE ||
        latest_lm_health_ == LaserMappingHealth::OVERLOADED_POST_ADJUSTMENT) { // OVERLOADED_POST_ADJUSTMENT added
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

    // If LM is healthy, check SR
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

    // Default or if states are mixed in a way not yet handled
    current_system_health_ = SystemHealth::UNKNOWN;
}


void AdaptiveParameterManager::processHealthAndAdjustParameters() {
    if (!current_adaptive_mode_.enabled) {
        RCLCPP_INFO(this->get_logger(), "Adaptive mode is disabled. No parameter adjustments will be made.");
        return;
    }

    determineSystemHealth(); // Update overall system health

    // Store previous parameters to check for changes later
    double prev_corner_filter = current_filter_parameter_corner_;
    double prev_surf_filter = current_filter_parameter_surf_;

    // Overload detection logic
    if (current_system_health_ == SystemHealth::LASER_MAPPING_ICP_UNSTABLE) {
        consecutive_icp_issue_warnings_++;
        consecutive_healthy_cycles_ = 0; // Reset healthy counter on any issue
        if (consecutive_icp_issue_warnings_ >= ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ && !overload_cooldown_active_) {
            RCLCPP_WARN(this->get_logger(), "Overload detected! Consecutive ICP issues reached threshold. Activating cooldown.");
            overload_cooldown_active_ = true;
            // Force a more conservative stance if overload is detected
            // This might mean increasing filter sizes significantly as a recovery measure
            current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_normal_ * 2.0); // Larger step to recover
            current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_normal_ * 2.0); // Larger step to recover
            // Update system health to reflect this forced recovery state if needed, or let next cycle handle it
            // Forcing parameters will be logged in applyParameterChanges if they changed.
        }
    } else if (current_system_health_ == SystemHealth::HEALTHY) {
        consecutive_icp_issue_warnings_ = 0; // Reset on any non-ICP-unstable state that is handled
        if (overload_cooldown_active_) {
            consecutive_healthy_cycles_++;
            if (consecutive_healthy_cycles_ >= HEALTHY_CYCLES_TO_RESET_COOLDOWN_) {
                RCLCPP_INFO(this->get_logger(), "System stable for %d cycles. Resetting overload cooldown.", consecutive_healthy_cycles_);
                overload_cooldown_active_ = false;
                consecutive_healthy_cycles_ = 0;
            }
        }
    } else {
        // For other non-healthy states that are not ICP_UNSTABLE, reset counters if appropriate
        // For example, if it's just SCAN_REGISTRATION_ISSUES, ICP itself isn't the problem.
        if (current_system_health_ != SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP) {
          // If it's not few features for ICP (which can lead to ICP unstable if not careful)
          // then reset icp issue counter.
           consecutive_icp_issue_warnings_ = 0;
        }
        consecutive_healthy_cycles_ = 0; // Reset healthy counter if not perfectly healthy
    }


    switch (current_system_health_) {
        case SystemHealth::LASER_MAPPING_FEW_FEATURES_FOR_ICP:
            RCLCPP_INFO(this->get_logger(), "System State: LASER_MAPPING_FEW_FEATURES_FOR_ICP. Adjusting for more features.");
            if (!overload_cooldown_active_) {
                current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_normal_);
                current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_normal_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Overload cooldown active, no reduction in filter sizes for FEW_FEATURES_ICP.");
            }
            // consecutive_icp_issue_warnings_ = 0; // Already handled above for non-ICP_UNSTABLE states
            break;

        case SystemHealth::LASER_MAPPING_ICP_UNSTABLE:
            RCLCPP_INFO(this->get_logger(), "System State: LASER_MAPPING_ICP_UNSTABLE. (%d consecutive warnings)", consecutive_icp_issue_warnings_);
            // The overload detection above already incremented consecutive_icp_issue_warnings_
            // and might have already taken action if threshold was met.
            // If overload cooldown is active, we generally want to be more conservative or try to increase filters.
            if (overload_cooldown_active_) {
                RCLCPP_INFO(this->get_logger(), "...Overload cooldown is active. Attempting to stabilize by ensuring filters are not too small or increasing them.");
                // If overload was just triggered, parameters might have been increased by the overload logic.
                // If cooldown is already active from a previous cycle, continue to be cautious.
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
            } else {
                // If not in cooldown, but ICP is unstable (and overload not yet triggered this cycle)
                if (latest_sr_health_ == ScanRegistrationHealth::LOW_RAW_POINTS ||
                    latest_sr_health_ == ScanRegistrationHealth::LOW_SHARP_FEATURES ||
                    latest_sr_health_ == ScanRegistrationHealth::LOW_FLAT_FEATURES) {
                    RCLCPP_INFO(this->get_logger(), "...and ScanRegistration has few features. Cautiously decreasing filter sizes.");
                    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, current_filter_parameter_corner_ - adjustment_step_small_);
                    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, current_filter_parameter_surf_ - adjustment_step_small_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "...and ScanRegistration is healthy. Increasing filter sizes to stabilize ICP.");
                    current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                    current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
                }
            }
            break;

        case SystemHealth::SCAN_REGISTRATION_ISSUES:
            RCLCPP_INFO(this->get_logger(), "System State: SCAN_REGISTRATION_ISSUES. Maintaining current LaserMapping filters.");
            // consecutive_icp_issue_warnings_ = 0; // Handled by general logic at the top
            break;

        case SystemHealth::HEALTHY:
            RCLCPP_INFO(this->get_logger(), "System State: HEALTHY. (%d consecutive healthy cycles)", consecutive_healthy_cycles_);
            // consecutive_icp_issue_warnings_ = 0; // Handled by general logic
            // overload_cooldown_active_ reset is handled by general logic based on consecutive_healthy_cycles_

            // Gently increase filter sizes to save resources if not in cooldown.
            // If cooldown was just reset, wait for next cycle to start probing.
            if (!overload_cooldown_active_ && consecutive_healthy_cycles_ == 0) { // Ensure cooldown is truly off
                 RCLCPP_INFO(this->get_logger(), "Probing for resource savings by increasing filter sizes.");
                current_filter_parameter_corner_ = std::min(max_filter_parameter_corner_, current_filter_parameter_corner_ + adjustment_step_small_);
                current_filter_parameter_surf_ = std::min(max_filter_parameter_surf_, current_filter_parameter_surf_ + adjustment_step_small_);
            } else if (overload_cooldown_active_) {
                RCLCPP_INFO(this->get_logger(), "Healthy, but overload cooldown is still active (%d/%d cycles). Holding parameters.", consecutive_healthy_cycles_, HEALTHY_CYCLES_TO_RESET_COOLDOWN_);
            }
            break;

        case SystemHealth::UNKNOWN:
        default:
            RCLCPP_INFO(this->get_logger(), "System State: UNKNOWN. No parameter adjustments.");
            // consecutive_icp_issue_warnings_ = 0; // Handled by general logic
            break;
    }

    // Ensure parameters are within bounds after any adjustment
    current_filter_parameter_corner_ = std::max(min_filter_parameter_corner_, std::min(max_filter_parameter_corner_, current_filter_parameter_corner_));
    current_filter_parameter_surf_ = std::max(min_filter_parameter_surf_, std::min(max_filter_parameter_surf_, current_filter_parameter_surf_));

    // Check if parameters actually changed
    if (std::abs(current_filter_parameter_corner_ - prev_corner_filter) > 1e-5 || // Use a small epsilon for float comparison
        std::abs(current_filter_parameter_surf_ - prev_surf_filter) > 1e-5) {
        RCLCPP_INFO(this->get_logger(), "Parameters changed. New Corner Filter: %.3f (was %.3f), New Surf Filter: %.3f (was %.3f)",
                    current_filter_parameter_corner_, prev_corner_filter,
                    current_filter_parameter_surf_, prev_surf_filter);
        applyParameterChanges();
    }
}

void AdaptiveParameterManager::applyParameterChanges() {
    // CONCEPTUAL INTEGRATION DETAILS:
    // In a real implementation, this method would use a ROS 2 parameter client
    // (e.g., rclcpp::SyncParametersClient or rclcpp::AsyncParametersClient)
    // to set the 'filter_parameter_corner' and 'filter_parameter_surf'
    // parameters on the '/laser_mapping_node'.
    //
    // The laserMapping node would need to:
    // 1. Declare these parameters (filter_parameter_corner, filter_parameter_surf)
    //    if not already done in a way that supports dynamic updates.
    // 2. Implement a mechanism to react to parameter changes. This could be:
    //    a. Periodically calling `this->get_parameter()` for these parameters in its main loop.
    //    b. Registering parameter event handlers (add_on_set_parameters_callback).
    //       This is the recommended ROS 2 way for dynamic parameters.
    //    When new parameter values are received, laserMapping would update its
    //    internal VoxelGrid filter leaf sizes.

    // Example using a hypothetical parameter client:
    /*
    if (!laser_mapping_param_client_) {
        // Initialize the parameter client if not already done
        // (typically in the constructor or a dedicated init method)
        laser_mapping_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "laser_mapping_node");
    }

    if (!laser_mapping_param_client_ || !laser_mapping_param_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "LaserMapping parameter service not available for setting parameters.");
        // Potentially revert parameter change intention or queue it
        return;
    }

    auto set_parameters_results = laser_mapping_param_client_->set_parameters({
        rclcpp::Parameter("laser_mapping_node.filter_parameter_corner", current_filter_parameter_corner_),
        rclcpp::Parameter("laser_mapping_node.filter_parameter_surf", current_filter_parameter_surf_)
        // Note: Parameter names might need to be fully qualified depending on node's parameter declaration
    });

    for (const auto &result : set_parameters_results) {
        if (!result.successful) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter on laser_mapping_node: %s", result.reason.c_str());
            // Handle failure: e.g., revert local values, retry, or enter a safe mode
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully set parameter on laser_mapping_node: %s to new value", result.name.c_str()); // This is not how you get name/value from result.
        }
    }
    // For set_parameters, the result is a vector of rcl_interfaces::msg::SetParametersResult.
    // Each element corresponds to one parameter in the request.
    // result.successful and result.reason are per-parameter.
    */

    RCLCPP_INFO(this->get_logger(), "[CONCEPTUAL] Applying new parameters to LaserMapping: Corner=%.3f, Surf=%.3f. See comments in code for integration details.",
                current_filter_parameter_corner_, current_filter_parameter_surf_);

    // After applying, we might want to monitor if this change leads to immediate negative effects.
    // The overload detection in step 4 will handle sustained issues.
}

} // namespace loam_adaptive_parameter_manager

// Basic main for testing purposes (conceptual, would not be part of the library itself)
/*
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto manager_node = std::make_shared<loam_adaptive_parameter_manager::AdaptiveParameterManager>();

    // Simulate health updates
    manager_node->updateLaserMappingHealth(loam_adaptive_parameter_manager::LaserMappingHealth::LOW_ICP_CORRESPONDENCES);
    manager_node->processHealthAndAdjustParameters();

    manager_node->updateLaserMappingHealth(loam_adaptive_parameter_manager::LaserMappingHealth::HEALTHY);
    manager_node->updateScanRegistrationHealth(loam_adaptive_parameter_manager::ScanRegistrationHealth::HEALTHY);
    manager_node->processHealthAndAdjustParameters();
    manager_node->processHealthAndAdjustParameters(); // Probe further

    rclcpp::spin(manager_node); // Keep node alive if it had timers or actual subscribers
    rclcpp::shutdown();
    return 0;
}
*/
