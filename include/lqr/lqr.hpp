#ifndef LQR_ROS2_NODE_PROJECT_LQR_NODE_HPP_
#define LQR_ROS2_NODE_PROJECT_LQR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

#define _USE_MATH_DEFINES

struct Vector {
    double x, y;
};

struct Vector3 {
    double x, y, z;
};

struct Point {
    double x, y;
};

struct Odometry {
    Point pose;
    double yaw;
};

struct Waypoint {
    Point pose;
    double tangent;
};

// KD-tree adaptor for nanoflann
struct PointCloud {
    std::vector<Point> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return (dim == 0) ? pts[idx].x : pts[idx].y;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

class LQR : public rclcpp::Node {
public:
    LQR();

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

#endif
