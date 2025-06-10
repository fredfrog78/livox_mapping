#ifndef ADAPTIVE_PARAMETER_MANAGER_TYPES_H
#define ADAPTIVE_PARAMETER_MANAGER_TYPES_H

namespace loam_adaptive_parameter_manager {

// Represents the health status from the scanRegistration nodes
enum class ScanRegistrationHealth {
    HEALTHY,
    LOW_RAW_POINTS,         // Warning: health.min_raw_points_for_feature_extraction
    LOW_SHARP_FEATURES,     // Warning: health.min_sharp_features
    LOW_FLAT_FEATURES       // Warning: health.min_flat_features
};

// Represents the health status from the laserMapping node
enum class LaserMappingHealth {
    HEALTHY,
    LOW_DOWNSAMPLED_CORNER_FEATURES, // Warning: health.min_downsampled_corner_features
    LOW_DOWNSAMPLED_SURF_FEATURES,   // Warning: health.min_downsampled_surf_features
    LOW_MAP_CORNER_POINTS_ICP,       // Warning: health.min_map_corner_points_for_icp
    LOW_MAP_SURF_POINTS_ICP,         // Warning: health.min_map_surf_points_for_icp
    LOW_ICP_CORRESPONDENCES,         // Warning: health.min_icp_correspondences
    HIGH_ICP_DELTA_ROTATION,         // Warning: health.max_icp_delta_rotation_deg
    HIGH_ICP_DELTA_TRANSLATION,      // Warning: health.max_icp_delta_translation_cm
    ICP_DEGENERATE                  // Warning: health.warn_on_icp_degeneracy
    // PIPELINE_STALLED removed, covered by SystemHealth::PIPELINE_FALLING_BEHIND
    // OVERLOADED_POST_ADJUSTMENT removed, covered by SystemHealth::LASER_MAPPING_OVERLOADED
};

// Overall system health summary
// Ordered roughly from most critical/specific to least critical/general
enum class SystemHealth {
    LASER_MAPPING_ICP_UNSTABLE,         // Critical mapping failure
    LASER_MAPPING_OVERLOADED,           // System is struggling, parameters already pushed
    PIPELINE_FALLING_BEHIND,            // Cannot keep up with real-time processing
    HIGH_CPU_LOAD,                      // Resource constraint: CPU
    LOW_MEMORY,                         // Resource constraint: Memory
    LASER_MAPPING_FEW_FEATURES_FOR_ICP, // Insufficient data for mapping
    SCAN_REGISTRATION_ISSUES,           // Upstream data processing issues
    HEALTHY,                            // All nominal
    UNKNOWN                             // Fallback or initial state
};

// Represents different operational strategies for the adaptive parameter manager
enum class AdaptiveProfile {
    PRIORITIZE_ACCURACY, // Default: Maximize ICP iterations, use smaller filter sizes, relax only when needed
    BALANCED,            // Try to balance accuracy with resource usage
    CONSERVE_RESOURCES   // Actively try to use fewer resources, potentially at cost of some accuracy
};

struct AdaptiveMode {
    bool enabled = true; // Master switch for adaptive behavior
    AdaptiveProfile profile = AdaptiveProfile::PRIORITIZE_ACCURACY; // Default profile

    // Could add specific thresholds here if they were to vary by profile,
    // but for now, let's keep thresholds as separate ROS parameters.
};

} // namespace loam_adaptive_parameter_manager

#endif // ADAPTIVE_PARAMETER_MANAGER_TYPES_H
