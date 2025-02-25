#include "lqr_node/lqr_node.hpp"

LQRNode::LQRNode() : Node("lqr_node") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&LQRNode::odometry_callback, this, std::placeholders::_1));
}

void LQRNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Posizione: x=%.2f, y=%.2f, z=%.2f",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRNode>());
    rclcpp::shutdown();
    return 0;
}
