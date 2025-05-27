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

    # Define the scanRegistration node
    scan_registration_node = Node(
        package='livox_mapping',
        executable='loam_scanRegistration', # Assuming executable name from CMakeLists.txt
        name='scanRegistration',
        output='screen'
    )

    # Define the laserMapping node
    laser_mapping_node = Node(
        package='livox_mapping',
        executable='loam_laserMapping', # Assuming executable name from CMakeLists.txt
        name='laserMapping',
        output='screen',
        parameters=[{
            'map_file_path': ' ', # Original value was an empty space string
            'filter_parameter_corner': 0.1,
            'filter_parameter_surf': 0.2
        }]
    )

    # Define the RViz node configuration
    rviz_config_file = os.path.join(livox_mapping_share_dir, 'rviz_cfg', 'loam_livox.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
        # launch_prefix=['nice'] # 'nice' is a linux command, not typically used directly in launch_prefix this way in ROS2. 
                                 # If niceness is required, it's better handled outside or via system configuration.
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_rviz_arg)
    ld.add_action(scan_registration_node)
    ld.add_action(laser_mapping_node)
    ld.add_action(rviz_node) # RViz node itself handles the IfCondition

    return ld
