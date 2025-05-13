from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the frontier exploration node
        Node(
            package='frontier_exploration',
            executable='frontier_exploration',
            name='frontier_exploration',
            parameters=[{
                'debug': True  # Enable debug mode for visualization
            }],
            output='screen'
        ),
        
        # Launch the pure pursuit node
        Node(
            package='frontier_exploration',
            executable='pure_pursuit',
            name='pure_pursuit',
            parameters=[{
                'debug': True  # Enable debug mode for visualization
            }],
            output='screen'
        )
    ]) 