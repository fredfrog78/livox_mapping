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
        default_value='true',
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

    # Set use_sim_time parameter
    set_use_sim_time = SetParameter(name='use_sim_time', value=False)

    # Node for hypothetical Mid360 driver
    # Replace 'livox_ros2_driver' and 'mid360_node' with actual package and executable
    node_mid360_driver = Node(
        package='livox_ros2_driver', # Placeholder
        executable='mid360_node',    # Placeholder
        name='mid360_driver_node',
        output='screen',
        parameters=[
            {'frame_id': LaunchConfiguration('frame_id')},
            # Assuming the driver uses these parameter names to set its output topics
            {'livox_lidar_frame_id': LaunchConfiguration('frame_id')}, # Common for livox driver
            {'publish_freq': 10.0}, # Example parameter
            # The driver itself would publish to topics specified by its internal params or command-line args.
            # For this example, we assume it's configured to publish to the topics
            # defined by point_cloud_topic and imu_topic launch arguments.
            # If the driver has parameters like "output_pointcloud_topic", they would be set here:
            # {'output_pointcloud_topic': LaunchConfiguration('point_cloud_topic')},
            # {'output_imu_topic': LaunchConfiguration('imu_topic')},
        ]
    )

    # Node for loam_scanRegistration
    node_scan_registration = Node(
        package='livox_mapping',
        executable='loam_scanRegistration',
        name='loam_scanRegistration',
        output='screen',
        remappings=[
            ('/livox/lidar', LaunchConfiguration('point_cloud_topic')),
            ('/livox/imu', LaunchConfiguration('imu_topic'))
        ]
    )

    # Node for loam_laserMapping
    node_laser_mapping = Node(
        package='livox_mapping',
        executable='loam_laserMapping',
        name='loam_laserMapping',
        output='screen'
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
        set_use_sim_time,
        node_mid360_driver, # Add the driver node
        node_scan_registration,
        node_laser_mapping,
        node_rviz
    ])
