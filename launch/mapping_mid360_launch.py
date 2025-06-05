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
            'publish_icp_correspondence_markers': LaunchConfiguration('publish_icp_correspondence_markers'),
            'publish_selected_feature_markers': LaunchConfiguration('publish_selected_feature_markers')
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
        declare_publish_icp_correspondence_markers_arg,
        declare_publish_selected_feature_markers_arg,
        node_scan_registration,
        node_laser_mapping,
        node_rviz
    ])
