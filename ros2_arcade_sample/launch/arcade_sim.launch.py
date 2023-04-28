from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_arcade_sample',
            executable='sim_node',
            name='sim_node',
        ),
        Node(
            package='ros2_arcade_sample',
            executable='controller',
            name='controller',
            parameters=[
                {'goal_x': 650},
                {'goal_y': 450}
            ],
        ),
    ])