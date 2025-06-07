import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz if true'
    )
    declare_point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/livox/lidar',
        description='Point cloud topic name for scanRegistration'
    )
    # IMU topic not directly used by these nodes but can be for other components
    # declare_imu_topic_arg = DeclareLaunchArgument(
    #     'imu_topic',
    #     default_value='/livox/imu',
    #     description='IMU topic name'
    # )
    # frame_id not directly used by these nodes, but good for reference
    # declare_frame_id_arg = DeclareLaunchArgument(
    #     'frame_id',
    #     default_value='lidar_frame',
    #     description='Lidar frame ID'
    # )
    declare_markers_icp_corr_arg = DeclareLaunchArgument(
        'markers_icp_corr',
        default_value='false',
        description='Enable/disable ICP correspondence debug markers for loam_laserMapping'
    )
    declare_markers_sel_features_arg = DeclareLaunchArgument(
        'markers_sel_features',
        default_value='false',
        description='Enable/disable selected feature debug markers for loam_laserMapping'
    )
    declare_enable_icp_debug_logs_arg = DeclareLaunchArgument(
        'enable_icp_debug_logs',
        default_value='false',
        description='Enable/disable console debug logging for ICP in loam_laserMapping'
    )

    # Declare health monitoring launch arguments for scanRegistration
    declare_sr_health_min_raw_points_arg = DeclareLaunchArgument(
        'sr_health_min_raw_points', default_value='10',
        description='Min raw points for feature extraction in scanRegistration'
    )
    declare_sr_health_min_sharp_features_arg = DeclareLaunchArgument(
        'sr_health_min_sharp_features', default_value='15',
        description='Min sharp features in scanRegistration'
    )
    declare_sr_health_min_flat_features_arg = DeclareLaunchArgument(
        'sr_health_min_flat_features', default_value='40',
        description='Min flat features in scanRegistration'
    )
    declare_sr_health_enable_warnings_arg = DeclareLaunchArgument(
        'sr_health_enable_warnings', default_value='true',
        description='Enable health warnings in scanRegistration'
    )

    # Declare health monitoring launch arguments for laserMapping
    declare_lm_health_min_downsampled_corner_features_arg = DeclareLaunchArgument(
        'lm_health_min_downsampled_corner_features', default_value='12',
        description='Min downsampled corner features in laserMapping'
    )
    declare_lm_health_min_downsampled_surf_features_arg = DeclareLaunchArgument(
        'lm_health_min_downsampled_surf_features', default_value='30',
        description='Min downsampled surf features in laserMapping'
    )
    declare_lm_health_min_map_corner_points_for_icp_arg = DeclareLaunchArgument(
        'lm_health_min_map_corner_points_for_icp', default_value='30',
        description='Min map corner points for ICP in laserMapping'
    )
    declare_lm_health_min_map_surf_points_for_icp_arg = DeclareLaunchArgument(
        'lm_health_min_map_surf_points_for_icp', default_value='100',
        description='Min map surf points for ICP in laserMapping'
    )
    declare_lm_health_min_icp_correspondences_arg = DeclareLaunchArgument(
        'lm_health_min_icp_correspondences', default_value='40',
        description='Min ICP correspondences in laserMapping'
    )
    declare_lm_health_max_icp_delta_rotation_deg_arg = DeclareLaunchArgument(
        'lm_health_max_icp_delta_rotation_deg', default_value='5.0',
        description='Max ICP delta rotation (deg) in laserMapping'
    )
    declare_lm_health_max_icp_delta_translation_cm_arg = DeclareLaunchArgument(
        'lm_health_max_icp_delta_translation_cm', default_value='20.0',
        description='Max ICP delta translation (cm) in laserMapping'
    )
    declare_lm_health_warn_on_icp_degeneracy_arg = DeclareLaunchArgument(
        'lm_health_warn_on_icp_degeneracy', default_value='true',
        description='Warn on ICP degeneracy in laserMapping'
    )
    declare_lm_health_enable_warnings_arg = DeclareLaunchArgument(
        'lm_health_enable_warnings', default_value='true',
        description='Enable health warnings in laserMapping'
    )

    # Declare Launch Arguments for AdaptiveParameterManager
    declare_apm_target_node_name_arg = DeclareLaunchArgument(
        'apm_target_node_name',
        default_value='loam_laserMapping', # Corrected to actual node name used below
        description='Target node name for AdaptiveParameterManager to control'
    )
    declare_apm_timer_period_sec_arg = DeclareLaunchArgument(
        'apm_timer_period_sec',
        default_value='1.0',
        description='Processing timer period in seconds for AdaptiveParameterManager'
    )

    # Node for loam_scanRegistration
    # Note: The executable is loam_scanRegistration, but the node name within C++ is scan_registration_node
    # The health topic will be /scan_registration/health_status
    node_scan_registration = Node(
        package='livox_mapping',
        executable='loam_scanRegistration',
        name='loam_scanRegistration', # This is the ROS node name that appears in `ros2 node list`
        output='screen',
        parameters=[
            {'health.min_raw_points_for_feature_extraction': LaunchConfiguration('sr_health_min_raw_points')},
            {'health.min_sharp_features': LaunchConfiguration('sr_health_min_sharp_features')},
            {'health.min_flat_features': LaunchConfiguration('sr_health_min_flat_features')},
            {'health.enable_health_warnings': LaunchConfiguration('sr_health_enable_warnings')}
        ],
        remappings=[ # Remap /livox_pcl0 to the desired input topic
            ('/livox_pcl0', LaunchConfiguration('point_cloud_topic'))
        ]
    )

    # Node for loam_laserMapping
    # Note: The executable is loam_laserMapping, node name in C++ is laser_mapping_node
    # Health topic will be /laser_mapping/health_status
    # Parameter client in APM will target 'loam_laserMapping' if that's its ROS node name
    node_laser_mapping = Node(
        package='livox_mapping',
        executable='loam_laserMapping',
        name='loam_laserMapping', # This is the ROS node name
        output='screen',
        parameters=[
            {'markers_icp_corr': LaunchConfiguration('markers_icp_corr')},
            {'markers_sel_features': LaunchConfiguration('markers_sel_features')},
            {'enable_icp_debug_logs': LaunchConfiguration('enable_icp_debug_logs')},
            # Dynamic parameters that APM might control
            # Ensure their defaults here are the system's startup defaults
            {'filter_parameter_corner': 0.2},
            {'filter_parameter_surf': 0.4},
            # Health parameters
            {'health.min_downsampled_corner_features': LaunchConfiguration('lm_health_min_downsampled_corner_features')},
            {'health.min_downsampled_surf_features': LaunchConfiguration('lm_health_min_downsampled_surf_features')},
            {'health.min_map_corner_points_for_icp': LaunchConfiguration('lm_health_min_map_corner_points_for_icp')},
            {'health.min_map_surf_points_for_icp': LaunchConfiguration('lm_health_min_map_surf_points_for_icp')},
            {'health.min_icp_correspondences': LaunchConfiguration('lm_health_min_icp_correspondences')},
            {'health.max_icp_delta_rotation_deg': LaunchConfiguration('lm_health_max_icp_delta_rotation_deg')},
            {'health.max_icp_delta_translation_cm': LaunchConfiguration('lm_health_max_icp_delta_translation_cm')},
            {'health.warn_on_icp_degeneracy': LaunchConfiguration('lm_health_warn_on_icp_degeneracy')},
            {'health.enable_health_warnings': LaunchConfiguration('lm_health_enable_warnings')}
        ]
    )

    # Node for AdaptiveParameterManager
    node_adaptive_parameter_manager = Node(
        package='livox_mapping',
        executable='adaptive_parameter_manager_node',
        name='adaptive_parameter_manager_node', # ROS Node name for APM
        output='screen',
        parameters=[
            # The target_node_name for APM's parameter client should be the ROS name of laserMapping
            {'target_node_name': 'loam_laserMapping'},
            {'timer_period_sec': LaunchConfiguration('apm_timer_period_sec')}
        ]
        # APM subscribes to:
        # /scan_registration/health_status (from loam_scanRegistration's C++ node name 'scan_registration_node')
        # /laser_mapping/health_status (from loam_laserMapping's C++ node name 'laser_mapping_node')
        # These default topic names are based on the C++ node names.
        # If the launch file 'name' fields for scanReg and laserMapping nodes were different
        # than their C++ default node names, and if health topics included the full node name,
        # then remappings might be needed here. But current setup seems fine.
    )

    # RViz node
    rviz_config_file = os.path.join(
        get_package_share_directory('livox_mapping'),
        'rviz_cfg',
        'loam_livox.rviz'
    )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_point_cloud_topic_arg,
        # declare_imu_topic_arg, # Not used by these nodes
        # declare_frame_id_arg, # Not used by these nodes
        declare_markers_icp_corr_arg,
        declare_markers_sel_features_arg,
        declare_enable_icp_debug_logs_arg,
        # Scan Registration health args
        declare_sr_health_min_raw_points_arg,
        declare_sr_health_min_sharp_features_arg,
        declare_sr_health_min_flat_features_arg,
        declare_sr_health_enable_warnings_arg,
        # Laser Mapping health args
        declare_lm_health_min_downsampled_corner_features_arg,
        declare_lm_health_min_downsampled_surf_features_arg,
        declare_lm_health_min_map_corner_points_for_icp_arg,
        declare_lm_health_min_map_surf_points_for_icp_arg,
        declare_lm_health_min_icp_correspondences_arg,
        declare_lm_health_max_icp_delta_rotation_deg_arg,
        declare_lm_health_max_icp_delta_translation_cm_arg,
        declare_lm_health_warn_on_icp_degeneracy_arg,
        declare_lm_health_enable_warnings_arg,
        # APM args
        declare_apm_target_node_name_arg,
        declare_apm_timer_period_sec_arg,
        # Nodes
        node_scan_registration,
        node_laser_mapping,
        node_adaptive_parameter_manager, # Added APM node
        node_rviz
    ])
