#include "lqr/lqr.hpp"

LQR::LQR() : Node("lqr_node") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&LQR::odometry_callback, this, std::placeholders::_1));
}

void LQR::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Posizione: x=%.2f, y=%.2f, z=%.2f",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);
}
