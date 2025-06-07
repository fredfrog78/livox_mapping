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
    ICP_DEGENERATE,                  // Warning: health.warn_on_icp_degeneracy
    PIPELINE_STALLED,                // Placeholder: If pipeline processing rate drops
    OVERLOADED_POST_ADJUSTMENT       // Internal state: If adjustments lead to sustained ICP issues
};

// Overall system health summary
enum class SystemHealth {
    UNKNOWN,
    HEALTHY,
    SCAN_REGISTRATION_ISSUES,
    LASER_MAPPING_FEW_FEATURES_FOR_ICP,
    LASER_MAPPING_ICP_UNSTABLE,
    LASER_MAPPING_OVERLOADED // Added based on plan step 4
};

struct AdaptiveMode {
    bool enabled = true; // Master switch for adaptive behavior
    // Could add profiles like 'CONSERVE_CPU', 'MAX_ACCURACY' in the future
};

} // namespace loam_adaptive_parameter_manager

#endif // ADAPTIVE_PARAMETER_MANAGER_TYPES_H
