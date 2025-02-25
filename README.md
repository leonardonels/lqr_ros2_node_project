# LQR_ros2_node_project

## :memo: TODO
in: [nav_msg/Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
out: [ackermann_msgs/AckermannDrive.msg](https://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDrive.html)

## :package: Prerequisite packages
> What we need is ros2 humble

## :gear: How to build & Run
```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/leonardonels/lqr_ros2_node_project.git
cd ~/ros2_ws
colcon build --packages-select lqr_ros2_node_project --symlink-install
source install/setup.bash
```
```commandline
ros2 launch lqr_ros2_node_project lqr_node_launch.py
```

## :notebook_with_decorative_cover: assumptions
The circuit is already loaded and wont be discovered
