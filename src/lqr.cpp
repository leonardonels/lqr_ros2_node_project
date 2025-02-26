#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lqr/lqr.hpp"
#include "nanoflann/nanoflann.hpp"

PointCloud get_trajectory(const std::string& trajectory_csv) 
{
    std::ifstream file(trajectory_csv);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening the file " + trajectory_csv);
    }
    
    std::string line;
    PointCloud cloud;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        
        if (!x_str.empty() && !y_str.empty() && x_str != "x") {
            try {
                cloud.pts.push_back({std::stod(x_str), std::stod(y_str)});
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " (" << e.what() << ")\n";
            }
        }
    }
    file.close();
    
    if (cloud.pts.empty()) {
        throw std::runtime_error("Error empty trajectory.");
    }
    
    return cloud;
}

size_t get_closest_point(const PointCloud& cloud, const Point& odometry_pose)
{
    if (cloud.pts.empty()) {
        throw std::runtime_error("Error empty pointcloud");
    }    
    
    //set query point = odometry_point
    double query_point[2] = { odometry_pose.x, odometry_pose.y };
    
    // Build KD-Tree
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2>;
    
    KDTree tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();
    
    // Query point
    std::vector<size_t> ret_index(1);
    std::vector<double> out_dist_sqr(1);
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    tree.findNeighbors(resultSet, query_point, nanoflann::SearchParameters(10));
    
    return ret_index[0];
}

LQR::LQR() : Node("lqr_node") {
    //std::string package_share_directory = ament_index_cpp::get_package_share_directory("lqr_ros2_node_project");
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", 10,
    std::bind(&LQR::odometry_callback, this, std::placeholders::_1));
}
    
void LQR::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    // Get Data
    Point odometry_pose = { msg->pose.pose.position.x, msg->pose.pose.position.y };
    RCLCPP_INFO(this->get_logger(), "odometry_pose: x=%.2f, y=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("lqr_ros2_node_project");
    /*RCLCPP_INFO(this->get_logger(), "%s\n", package_share_directory.c_str());*/
    std::string trajectory_csv = "/vallelunga1_circuit.csv";
    PointCloud cloud = get_trajectory(package_share_directory+trajectory_csv);

    // Save the actual time to compute the time needed for the execution later
    auto start = std::chrono::high_resolution_clock::now();

    // Find closest point to trajectory using KD-Tree from NanoFLANN
    size_t closest_point_index = get_closest_point(cloud, odometry_pose);
    Point closest_point = cloud.pts[closest_point_index];
    RCLCPP_INFO(this->get_logger(), "Closest Point: x=%.2f, y=%.2f", closest_point.x, closest_point.y);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "duration: %ld ms", duration);
}
