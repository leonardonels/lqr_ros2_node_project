from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="LQR_ros2_node_project",
            executable="lqr_node",
            name="lqr_node",
            output="screen",
        )
    ])
