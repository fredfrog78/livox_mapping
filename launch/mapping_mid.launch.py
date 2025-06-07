import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of the livox_mapping package
    livox_mapping_share_dir = get_package_share_directory('livox_mapping')

    # Declare the rviz launch argument
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz austomatically with the launch file.'
    )

    # Declare health monitoring launch arguments
    declare_sr_health_min_raw_points_arg = DeclareLaunchArgument(
        'sr_health_min_raw_points', default_value='10',
        description='Min raw points for feature extraction in scanRegistration'
    )
    declare_sr_health_min_sharp_features_arg = DeclareLaunchArgument(
        'sr_health_min_sharp_features', default_value='20',
        description='Min sharp features in scanRegistration'
    )
    declare_sr_health_min_flat_features_arg = DeclareLaunchArgument(
        'sr_health_min_flat_features', default_value='50',
        description='Min flat features in scanRegistration'
    )
    declare_sr_health_enable_warnings_arg = DeclareLaunchArgument(
        'sr_health_enable_warnings', default_value='true',
        description='Enable health warnings in scanRegistration'
    )

    declare_lm_health_min_downsampled_corner_features_arg = DeclareLaunchArgument(
        'lm_health_min_downsampled_corner_features', default_value='10',
        description='Min downsampled corner features in laserMapping'
    )
    declare_lm_health_min_downsampled_surf_features_arg = DeclareLaunchArgument(
        'lm_health_min_downsampled_surf_features', default_value='30',
        description='Min downsampled surf features in laserMapping'
    )
    declare_lm_health_min_map_corner_points_for_icp_arg = DeclareLaunchArgument(
        'lm_health_min_map_corner_points_for_icp', default_value='50',
        description='Min map corner points for ICP in laserMapping'
    )
    declare_lm_health_min_map_surf_points_for_icp_arg = DeclareLaunchArgument(
        'lm_health_min_map_surf_points_for_icp', default_value='200',
        description='Min map surf points for ICP in laserMapping'
    )
    declare_lm_health_min_icp_correspondences_arg = DeclareLaunchArgument(
        'lm_health_min_icp_correspondences', default_value='75',
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
        default_value='laserMapping', # Target the laserMapping node as named below
        description='Target node name for AdaptiveParameterManager to control'
    )
    declare_apm_timer_period_sec_arg = DeclareLaunchArgument(
        'apm_timer_period_sec',
        default_value='1.0',
        description='Processing timer period in seconds for AdaptiveParameterManager'
    )

    # Define the scanRegistration node
    # Its C++ node name is 'scan_registration_node', publishing health to /scan_registration/health_status
    scan_registration_node = Node(
        package='livox_mapping',
        executable='loam_scanRegistration',
        name='scanRegistration', # ROS Node name
        output='screen',
        parameters=[
            {'health.min_raw_points_for_feature_extraction': LaunchConfiguration('sr_health_min_raw_points')},
            {'health.min_sharp_features': LaunchConfiguration('sr_health_min_sharp_features')},
            {'health.min_flat_features': LaunchConfiguration('sr_health_min_flat_features')},
            {'health.enable_health_warnings': LaunchConfiguration('sr_health_enable_warnings')}
        ]
    )

    # Define the laserMapping node
    # Its C++ node name is 'laser_mapping_node', publishing health to /laser_mapping/health_status
    laser_mapping_node = Node(
        package='livox_mapping',
        executable='loam_laserMapping',
        name='laserMapping', # ROS Node name
        output='screen',
        parameters=[
            {'map_file_path': ' '}, # Existing parameter
            # Ensure dynamically controlled parameters have explicit defaults here
            {'filter_parameter_corner': 0.1},
            {'filter_parameter_surf': 0.2},
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
        name='adaptive_parameter_manager_node',
        output='screen',
        parameters=[
            {'target_node_name': LaunchConfiguration('apm_target_node_name')},
            {'timer_period_sec': LaunchConfiguration('apm_timer_period_sec')}
        ]
        # No remappings needed as scanRegistration node here is 'scanRegistration' (C++ name 'scan_registration_node')
        # and APM defaults to /scan_registration/health_status.
    )

    # Define the RViz node configuration
    rviz_config_file = os.path.join(livox_mapping_share_dir, 'rviz_cfg', 'loam_livox.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_rviz_arg)
    # Add declared health arguments to LaunchDescription
    ld.add_action(declare_sr_health_min_raw_points_arg)
    ld.add_action(declare_sr_health_min_sharp_features_arg)
    ld.add_action(declare_sr_health_min_flat_features_arg)
    ld.add_action(declare_sr_health_enable_warnings_arg)
    ld.add_action(declare_lm_health_min_downsampled_corner_features_arg)
    ld.add_action(declare_lm_health_min_downsampled_surf_features_arg)
    ld.add_action(declare_lm_health_min_map_corner_points_for_icp_arg)
    ld.add_action(declare_lm_health_min_map_surf_points_for_icp_arg)
    ld.add_action(declare_lm_health_min_icp_correspondences_arg)
    ld.add_action(declare_lm_health_max_icp_delta_rotation_deg_arg)
    ld.add_action(declare_lm_health_max_icp_delta_translation_cm_arg)
    ld.add_action(declare_lm_health_warn_on_icp_degeneracy_arg)
    ld.add_action(declare_lm_health_enable_warnings_arg)
    # Add APM launch arguments
    ld.add_action(declare_apm_target_node_name_arg)
    ld.add_action(declare_apm_timer_period_sec_arg)

    ld.add_action(scan_registration_node)
    ld.add_action(laser_mapping_node)
    ld.add_action(node_adaptive_parameter_manager) # Added APM node
    ld.add_action(rviz_node)

    return ld
