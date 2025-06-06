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
        default_value='false', # Changed to false
        description='Launch RViz if true'
    )
    declare_point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/livox/lidar',
        description='Point cloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu',
        description='IMU topic name'
    )
    declare_frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar_frame',
        description='Lidar frame ID'
    )
    declare_markers_icp_corr_arg = DeclareLaunchArgument(
        'markers_icp_corr',
        default_value='false',
        description='Enable/disable ICP correspondence debug markers (markers_icp_corr) for loam_laserMapping'
    )
    declare_markers_sel_features_arg = DeclareLaunchArgument(
        'markers_sel_features',
        default_value='false',
        description='Enable/disable selected feature debug markers (markers_sel_features) for loam_laserMapping'
    )
    declare_enable_icp_debug_logs_arg = DeclareLaunchArgument(
        'enable_icp_debug_logs',
        default_value='false',
        description='Enable/disable console debug logging for ICP in loam_laserMapping'
    )

    # Node for loam_scanRegistration
    node_scan_registration = Node(
        package='livox_mapping',
        executable='loam_scanRegistration',
        name='loam_scanRegistration',
        output='screen',
        remappings=[
            ('/livox_pcl0', LaunchConfiguration('point_cloud_topic')), # Changed '/livox/lidar' to '/livox_pcl0'
            ('/livox/imu', LaunchConfiguration('imu_topic'))
        ]
    )

    # Node for loam_laserMapping
    node_laser_mapping = Node(
        package='livox_mapping',
        executable='loam_laserMapping',
        name='loam_laserMapping',
        output='screen',
        parameters=[{
            'markers_icp_corr': LaunchConfiguration('markers_icp_corr'),
            'markers_sel_features': LaunchConfiguration('markers_sel_features'),
            'enable_icp_debug_logs': LaunchConfiguration('enable_icp_debug_logs')
        }]
        # No direct remapping of /livox/lidar or /livox/imu here,
        # as it consumes processed topics from scanRegistration
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
        declare_imu_topic_arg,
        declare_frame_id_arg,
        declare_markers_icp_corr_arg,
        declare_markers_sel_features_arg,
        declare_enable_icp_debug_logs_arg,
        node_scan_registration,
        node_laser_mapping,
        node_rviz
    ])
