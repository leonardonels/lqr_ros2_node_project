from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lqr_ros2_node_project",
            executable="lqr_node",
            name="lqr_node",
            output="screen",
        )
    ])
