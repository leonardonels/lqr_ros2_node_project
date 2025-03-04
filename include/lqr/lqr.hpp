#ifndef LQR_ROS2_NODE_PROJECT_LQR_NODE_HPP_
#define LQR_ROS2_NODE_PROJECT_LQR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <eigen3/Eigen/Geometry>
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
    // callbacks
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void partial_trajectory_callback(const visualization_msgs::msg::Marker traj); // during the first lap we work with the partial trajectory published by the local planner
    // void global_trajectory_callback(); TO BE IMPLEMENTED IN THE FUTURE

    // methods
    void initialize();
    void load_parameters();
    Eigen::Vector4d find_optimal_control_vector(double speed);
    double calculate_throttle();

    // pubs and subs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr m_partial_traj_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_debug_odom_pub;
    
    //topics
    std::string m_odom_topic;
    std::string m_partial_traj_topic;
    std::string m_debug_odom_topic;
   
    // class members
    std::vector<std::string> m_raw_vectors_k;
    std::vector<std::pair<double, std::vector<double>>> m_k_pair;
    PointCloud m_cloud;
    bool m_first_lap; // for now we statically decide if we want to use the partial trajectory or we want to use the global trajectory
    bool m_loaded;
    bool m_DEBUG;

    // car physical parameters
    double m_mass;
    double front_length;
    double rear_length;
    double C_alpha_front;
    double C_alpha_rear;
};

#endif
