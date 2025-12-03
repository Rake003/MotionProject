#include "motion_projection/mesh_visualization.hpp"

namespace motion_projection
{

// ======================================================================
// Publish mesh as green wireframe (unchanged)
// ======================================================================
void publishMeshVisualization(
    const reconstruction::ReconState& recon,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    rclcpp::Node* node)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    visualization_msgs::msg::Marker mesh;
    mesh.header.frame_id = frame_id;
    mesh.header.stamp = node->now();
    mesh.ns = "mesh";
    mesh.id = 0;
    mesh.type = visualization_msgs::msg::Marker::LINE_LIST;
    mesh.action = visualization_msgs::msg::Marker::ADD;
    mesh.scale.x = 0.001;

    mesh.color.r = 0.0;
    mesh.color.g = 1.0;
    mesh.color.b = 0.0;
    mesh.color.a = 0.3;  // More transparent

    for (const auto& tri : recon.mesh_->triangles_) {
        const auto& v0 = recon.mesh_->vertices_[tri[0]];
        const auto& v1 = recon.mesh_->vertices_[tri[1]];
        const auto& v2 = recon.mesh_->vertices_[tri[2]];

        geometry_msgs::msg::Point p0, p1, p2;
        p0.x = v0.x(); p0.y = v0.y(); p0.z = v0.z();
        p1.x = v1.x(); p1.y = v1.y(); p1.z = v1.z();
        p2.x = v2.x(); p2.y = v2.y(); p2.z = v2.z();

        mesh.points.push_back(p0); mesh.points.push_back(p1);
        mesh.points.push_back(p1); mesh.points.push_back(p2);
        mesh.points.push_back(p2); mesh.points.push_back(p0);
    }

    marker_array.markers.push_back(mesh);
    pub->publish(marker_array);
}

// ======================================================================
// NEW: Publish only candidate triangles + selected triangle + its normal
// ======================================================================
void publishSelectedTrianglesVisualization(
    const reconstruction::ReconState& recon,
    const std::vector<int>& candidate_triangles,
    int selected_triangle,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    rclcpp::Node* node)
{
    visualization_msgs::msg::MarkerArray arr;

    // Clear old markers
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    // 1. Highlight candidate triangles (yellow wireframe)
    visualization_msgs::msg::Marker candidates;
    candidates.header.frame_id = frame_id;
    candidates.header.stamp = node->now();
    candidates.ns = "candidate_triangles";
    candidates.id = 0;
    candidates.type = visualization_msgs::msg::Marker::LINE_LIST;
    candidates.action = visualization_msgs::msg::Marker::ADD;
    candidates.scale.x = 0.002;  // Thicker lines
    candidates.color.r = 1.0;
    candidates.color.g = 1.0;
    candidates.color.b = 0.0;
    candidates.color.a = 0.6;

    for (int tri_idx : candidate_triangles) {
        const auto& tri = recon.mesh_->triangles_[tri_idx];
        const auto& v0 = recon.mesh_->vertices_[tri[0]];
        const auto& v1 = recon.mesh_->vertices_[tri[1]];
        const auto& v2 = recon.mesh_->vertices_[tri[2]];

        geometry_msgs::msg::Point p0, p1, p2;
        p0.x = v0.x(); p0.y = v0.y(); p0.z = v0.z();
        p1.x = v1.x(); p1.y = v1.y(); p1.z = v1.z();
        p2.x = v2.x(); p2.y = v2.y(); p2.z = v2.z();

        candidates.points.push_back(p0); candidates.points.push_back(p1);
        candidates.points.push_back(p1); candidates.points.push_back(p2);
        candidates.points.push_back(p2); candidates.points.push_back(p0);
    }

    if (!candidates.points.empty()) {
        arr.markers.push_back(candidates);
    }

    // 2. Highlight selected triangle (red filled)
    if (selected_triangle >= 0 && selected_triangle < (int)recon.mesh_->triangles_.size()) {
        visualization_msgs::msg::Marker selected;
        selected.header.frame_id = frame_id;
        selected.header.stamp = node->now();
        selected.ns = "selected_triangle";
        selected.id = 1;
        selected.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        selected.action = visualization_msgs::msg::Marker::ADD;
        selected.scale.x = 1.0;
        selected.scale.y = 1.0;
        selected.scale.z = 1.0;
        selected.color.r = 1.0;
        selected.color.g = 0.0;
        selected.color.b = 0.0;
        selected.color.a = 0.7;

        const auto& tri = recon.mesh_->triangles_[selected_triangle];
        const auto& v0 = recon.mesh_->vertices_[tri[0]];
        const auto& v1 = recon.mesh_->vertices_[tri[1]];
        const auto& v2 = recon.mesh_->vertices_[tri[2]];

        geometry_msgs::msg::Point p0, p1, p2;
        p0.x = v0.x(); p0.y = v0.y(); p0.z = v0.z();
        p1.x = v1.x(); p1.y = v1.y(); p1.z = v1.z();
        p2.x = v2.x(); p2.y = v2.y(); p2.z = v2.z();

        selected.points.push_back(p0);
        selected.points.push_back(p1);
        selected.points.push_back(p2);

        arr.markers.push_back(selected);

        // 3. Show normal of selected triangle (large red arrow)
        visualization_msgs::msg::Marker normal_arrow;
        normal_arrow.header.frame_id = frame_id;
        normal_arrow.header.stamp = node->now();
        normal_arrow.ns = "selected_normal";
        normal_arrow.id = 2;
        normal_arrow.type = visualization_msgs::msg::Marker::ARROW;
        normal_arrow.action = visualization_msgs::msg::Marker::ADD;

        // Triangle center
        Eigen::Vector3d center = (v0 + v1 + v2) / 3.0;
        Eigen::Vector3d normal = recon.mesh_->triangle_normals_[selected_triangle];

        normal_arrow.points.resize(2);
        normal_arrow.points[0].x = center.x();
        normal_arrow.points[0].y = center.y();
        normal_arrow.points[0].z = center.z();

        Eigen::Vector3d end = center + 0.05 * normal;  // 5cm arrow
        normal_arrow.points[1].x = end.x();
        normal_arrow.points[1].y = end.y();
        normal_arrow.points[1].z = end.z();

        normal_arrow.scale.x = 0.005;  // Shaft
        normal_arrow.scale.y = 0.008;  // Head

        normal_arrow.color.r = 1.0;
        normal_arrow.color.g = 0.0;
        normal_arrow.color.b = 0.0;
        normal_arrow.color.a = 1.0;

        arr.markers.push_back(normal_arrow);
    }

    pub->publish(arr);
}

// ======================================================================
// NEW: Publish current robot position as a sphere marker
// ======================================================================
void publishCurrentPositionMarker(
    const Eigen::Vector3d& position,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    rclcpp::Node* node)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.ns = "current_position";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;  // 1cm diameter
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    pub->publish(marker);
}

// ======================================================================
// Keep old function for compatibility (but unused now)
// ======================================================================
void publishNormalsVisualization(
    const reconstruction::ReconState& recon,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    rclcpp::Node* node,
    int current_tri,
    double arrow_length)
{
    // This function is now replaced by publishSelectedTrianglesVisualization
    // Keep it for backward compatibility but do nothing
    (void)recon;
    (void)frame_id;
    (void)pub;
    (void)node;
    (void)current_tri;
    (void)arrow_length;
}

// ======================================================================
// Trajectory (unchanged)
// ======================================================================
void publishTrajectory(
    const std::vector<Eigen::Vector3d>& trajectory,
    const std::string& frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    rclcpp::Node* node)
{
    visualization_msgs::msg::Marker strip;
    strip.header.frame_id = frame_id;
    strip.header.stamp = node->now();
    strip.ns = "trajectory";
    strip.id = 0;
    strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    strip.action = visualization_msgs::msg::Marker::ADD;

    strip.scale.x = 0.003;

    strip.color.r = 1.0;
    strip.color.g = 1.0;
    strip.color.b = 0.0;
    strip.color.a = 1.0;

    for (const auto& pt : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        strip.points.push_back(p);
    }

    pub->publish(strip);
}

} // namespace motion_projection