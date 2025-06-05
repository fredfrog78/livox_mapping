import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false', # Changed to false
        description='Launch RViz if true'
    )
    declare_publish_icp_correspondence_markers_arg = DeclareLaunchArgument(
        'publish_icp_correspondence_markers',
        default_value='false',
        description='Enable/disable ICP correspondence debug markers for loam_laserMapping'
    )
    declare_publish_selected_feature_markers_arg = DeclareLaunchArgument(
        'publish_selected_feature_markers',
        default_value='false',
        description='Enable/disable selected feature debug markers for loam_laserMapping'
    )

    # Set use_sim_time parameter
    set_use_sim_time = SetParameter(name='use_sim_time', value=False)

    # Node for livox_repub
    node_livox_repub = Node(
        package='livox_mapping',
        executable='livox_repub', # Executable name for livox_repub
        name='livox_repub',
        output='screen'
    )

    # Node for loam_scanRegistration
    node_scan_registration = Node(
        package='livox_mapping',
        executable='loam_scanRegistration', # Executable name for mid-range LiDAR
        name='loam_scanRegistration',
        output='screen'
    )

    # Node for loam_laserMapping
    node_laser_mapping = Node(
        package='livox_mapping',
        executable='loam_laserMapping', 
        name='loam_laserMapping',
        output='screen',
        parameters=[{
            'publish_icp_correspondence_markers': LaunchConfiguration('publish_icp_correspondence_markers'),
            'publish_selected_feature_markers': LaunchConfiguration('publish_selected_feature_markers')
        }]
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
        # prefix=['nice'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_publish_icp_correspondence_markers_arg,
        declare_publish_selected_feature_markers_arg,
        set_use_sim_time,
        node_livox_repub,
        node_scan_registration,
        node_laser_mapping,
        node_rviz
    ])
