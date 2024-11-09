from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collision_detection',
            executable='demo_torch_node',
            name='demo',
            output='screen',
            # parameters=[{'param_name': 'param_value'}]
        ),
    ])