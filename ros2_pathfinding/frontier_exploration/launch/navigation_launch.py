import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('frontier_exploration')
    
    # Create our own launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    # Specify the actions
    start_nav2_cmd = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map': map_file,
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_cmd)
    
    # Add the nodes to launch
    ld.add_action(start_nav2_cmd)
    
    return ld 