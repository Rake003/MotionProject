#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace helper_functions
{
    std::vector<Eigen::Vector3d> generate_sphere_pcd(double sphere_radius, const Eigen::Vector3d& centre_point, int num_points);
    sensor_msgs::msg::PointCloud2 eigenToPointCloud2(const std::vector<Eigen::Vector3d>& points);
    
     // ------------------------------------------------------
    // Mesh result structure (vertices + triangles)
    // ------------------------------------------------------
    struct MeshResult
    {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::array<int, 3>> triangles;
    };

    // ------------------------------------------------------
    // Triangulation using Open3D BPA
    // ------------------------------------------------------
    MeshResult triangulate_with_open3d(const std::vector<Eigen::Vector3d>& pcd, const std::string& output_ply_path);

}
