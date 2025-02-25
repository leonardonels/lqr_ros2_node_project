#ifndef LQR_ROS2_NODE_PROJECT_LQR_NODE_HPP_
#define LQR_ROS2_NODE_PROJECT_LQR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class LQR : public rclcpp::Node {
public:
    LQR();

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

#endif
