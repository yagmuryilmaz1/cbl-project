from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the map synchronization node
        Node(
            package='frontier_exploration',
            executable='map_sync',
            name='map_sync',
            output='screen'
        )
    ]) 