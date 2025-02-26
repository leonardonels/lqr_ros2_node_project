#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <filesystem>
#include <math.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lqr/lqr.hpp"
#include "nanoflann/nanoflann.hpp"

Vector3 crossProduct(const Vector3& A, const Vector3& B) {
    return { 
        0, 
        0, 
        A.x * B.y - A.y * B.x
    };
}

Point subtract(const Point &a, const Point &b) {
    return {a.x - b.x, a.y - b.y};
}

Point normalize(const Point &p) {
    double len = std::sqrt(p.x * p.x + p.y * p.y);
    if (len == 0) return {0, 0};  // Prevent division by zero
    return {p.x / len, p.y / len};
}

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

double distance(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double signed_distance(double Ax, double Ay, double Bx, double By, double theta) {
    
    Vector3 A = { cos(theta), sin(theta), 0 };
    
    Vector3 B = { Bx - Ax, By - Ay, 0 };
    
    Vector3 cross = crossProduct(A, B);
    
    return cross.z/std::abs(cross.z);
}

double get_angular_deviation(double angle1, double angle2) {
    // Compute the raw difference, then shift by π.
    double diff = angle2 - angle1 + M_PI;
    
    // Use fmod to wrap the value into the range [0, 2π)
    diff = std::fmod(diff, 2 * M_PI);
    
    // fmod can return a negative result; adjust if necessary.
    if (diff < 0)
        diff += 2 * M_PI;
    
    // Shift back by π to get a value in [-π, π]
    diff -= M_PI;
    
    // Return the absolute value to get the magnitude in [0, π]
    //return std::abs(diff);
    return diff;
}

std::vector<double> get_tangent_angles(std::vector<Point> points)
{

    std::vector<double> tangent_angles(points.size());
    
    if (points.size() >= 2) {
        // First point: forward difference.
        Point diff = subtract(points[1], points[0]);
        Point tanVec = normalize(diff);
        tangent_angles[0] = std::atan2(tanVec.y, tanVec.x);

        // Last point: backward difference.
        diff = subtract(points.back(), points[points.size() - 2]);
        tanVec = normalize(diff);
        tangent_angles.back() = std::atan2(tanVec.y, tanVec.x);
    }

     for (size_t i = 1; i < points.size() - 1; ++i) {
        double d1 = distance(points[i], points[i - 1]);
        double d2 = distance(points[i + 1], points[i]);
        double ds = d1 + d2;  // Total distance over the two segments

        if (ds == 0) {
            tangent_angles[i] = 0;  // Fallback if points coincide
        } else {
            // Compute the central difference divided by the total arc length.
            Point diff = {
                (points[i + 1].x - points[i - 1].x) / ds,
                (points[i + 1].y - points[i - 1].y) / ds
            };
            Point tanVec = normalize(diff);
            tangent_angles[i] = std::atan2(tanVec.y, tanVec.x);
        }
    }

    return tangent_angles;
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
    Odometry odometry = {odometry_pose, msg->pose.pose.orientation.x};
    RCLCPP_INFO(this->get_logger(), "odometry_pose: x=%.2f, y=%.2f", odometry_pose.x, odometry_pose.y);
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("lqr_ros2_node_project");
    /*RCLCPP_INFO(this->get_logger(), "%s\n", package_share_directory.c_str());*/
    std::string trajectory_csv = "/vallelunga1_circuit.csv";
    PointCloud cloud = get_trajectory(package_share_directory+trajectory_csv);

    // Save the actual time to compute the time needed for the execution later
    auto start = std::chrono::high_resolution_clock::now();

    // Find closest point to trajectory using KD-Tree from NanoFLANN
    size_t closest_point_index = get_closest_point(cloud, odometry_pose);
    Point closest_point = cloud.pts[closest_point_index];

    // Calculate lateral deviation as distance between two points
    double lateral_deviation = distance(odometry_pose, closest_point);

    // I have found the closest point on the trajectory to the odometry pose but I don't trust the result so I check if the previous or next point are closer to the odometry
    while(1)
    {
        if(closest_point_index > 0 && closest_point_index < cloud.pts.size() - 1)
        {
            double previuous_point_lateral_deviation = distance(odometry_pose, cloud.pts[closest_point_index - 1]);
            double next_point_lateral_deviation = distance(odometry_pose, cloud.pts[closest_point_index + 1]);
            if(previuous_point_lateral_deviation < lateral_deviation)
            {
                closest_point_index = closest_point_index - 1;
                closest_point = cloud.pts[closest_point_index];
                lateral_deviation = previuous_point_lateral_deviation;
            }
            else if(next_point_lateral_deviation < lateral_deviation)
            {
                closest_point_index = closest_point_index + 1;
                closest_point = cloud.pts[closest_point_index];
                lateral_deviation = next_point_lateral_deviation;
            }
            else if(next_point_lateral_deviation > lateral_deviation && previuous_point_lateral_deviation > lateral_deviation)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Closest Point: x=%.2f, y=%.2f", closest_point.x, closest_point.y);

    // At this point I have the closest point on the trajectory and the lateral deviation from the odometry to the trajectory
    // Now I need to find the angular deviation between the odometry and the trajectory

    // Compute for every point on the trajectory the tangent angle
    std::vector<double> points_tangents = get_tangent_angles(cloud.pts); 
    double closest_point_tangent = points_tangents[closest_point_index];

    double lateral_position = signed_distance(closest_point.x, closest_point.y, odometry_pose.x, odometry_pose.y, closest_point_tangent);
    lateral_deviation*=lateral_position;
    RCLCPP_INFO(this->get_logger(), "lateral deviation: %.2f", lateral_deviation);

    // Finally calculate the angular deviation between the odometry and the closest point on the trajectory
    double angular_deviation = get_angular_deviation(closest_point_tangent, odometry.yaw);
    RCLCPP_INFO(this->get_logger(), "angular deviation: %.2f", angular_deviation);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "duration: %ld ms", duration);
}
