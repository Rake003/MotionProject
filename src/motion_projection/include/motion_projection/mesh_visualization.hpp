#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>

#include "reconstruction/ReconState.hpp"

namespace motion_projection
{

// -------------------------------------------------------------
// Publish wireframe mesh (all triangles, semi-transparent)
// -------------------------------------------------------------
void publishMeshVisualization(
    const reconstruction::ReconState& recon,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    rclcpp::Node* node);

// -------------------------------------------------------------
// NEW: Publish only candidate triangles + selected triangle + normal
// Shows:
//   - Yellow wireframe for all candidate triangles
//   - Red filled triangle for the selected one
//   - Red arrow for the selected triangle's normal
// -------------------------------------------------------------
void publishSelectedTrianglesVisualization(
    const reconstruction::ReconState& recon,
    const std::vector<int>& candidate_triangles,
    int selected_triangle,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    rclcpp::Node* node);

// -------------------------------------------------------------
// NEW: Publish current robot position as a blue sphere
// -------------------------------------------------------------
void publishCurrentPositionMarker(
    const Eigen::Vector3d& position,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    rclcpp::Node* node);

// -------------------------------------------------------------
// OLD: Publish all triangle normals (deprecated, kept for compatibility)
// -------------------------------------------------------------
void publishNormalsVisualization(
    const reconstruction::ReconState& recon,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    rclcpp::Node* node,
    int current_tri = -1,
    double arrow_length = 0.05);

// -------------------------------------------------------------
// Publish trajectory of projection points
// -------------------------------------------------------------
void publishTrajectory(
    const std::vector<Eigen::Vector3d>& trajectory,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    rclcpp::Node* node);

} // namespace motion_projection