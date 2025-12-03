#include "motion_projection/helper_functions.hpp"
#include <cmath>
#include <open3d/Open3D.h>

namespace helper_functions
{

    std::vector<Eigen::Vector3d> generate_sphere_pcd(double sphere_radius, const Eigen::Vector3d& centre_point, int num_points)
    {
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(num_points);

        const double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
        const double golden_angle = 2.0 * M_PI * (1.0 - 1.0 / golden_ratio);

        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1);

            // y ranges from -1 to 1
            double y = 1.0 - 2.0 * t;

            // radius of circle at height y
            double r = std::sqrt(1.0 - y * y);

            // angle incremented using golden angle
            double theta = golden_angle * i;

            double x = std::cos(theta) * r;
            double z = std::sin(theta) * r;

            // scale to sphere radius & shift to centre
            Eigen::Vector3d pt = centre_point + sphere_radius * Eigen::Vector3d(x, y, z);

            pts.push_back(pt);
        }

        return pts;
    }

    sensor_msgs::msg::PointCloud2 eigenToPointCloud2(const std::vector<Eigen::Vector3d>& points)
    {
        sensor_msgs::msg::PointCloud2 msg;

        msg.header.frame_id = "base_link"; // or "world"
        msg.height = 1;
        msg.width  = points.size();
        msg.is_dense = true;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(
            3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT64,
            "y", 1, sensor_msgs::msg::PointField::FLOAT64,
            "z", 1, sensor_msgs::msg::PointField::FLOAT64);

        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<double> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<double> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<double> iter_z(msg, "z");

        for (auto &p : points)
        {
            *iter_x = p[0];
            *iter_y = p[1];
            *iter_z = p[2];

            ++iter_x; ++iter_y; ++iter_z;
        }

        return msg;
    }

    helper_functions::MeshResult triangulate_with_open3d(const std::vector<Eigen::Vector3d>& pcd, const std::string& output_ply_path)
    {
        using namespace open3d;

        // PointCloud
        auto o3d_pcd = std::make_shared<geometry::PointCloud>();
        o3d_pcd->points_.reserve(pcd.size());
        for (auto &p : pcd) o3d_pcd->points_.push_back(Eigen::Vector3d(p.x(), p.y(), p.z()));

        // Estimate normals
        o3d_pcd->EstimateNormals(geometry::KDTreeSearchParamKNN(30));
        // Optionally orient normals outward from center:
        // o3d_pcd->OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, 0.0)); 

        // Ball-Pivoting
        std::vector<double> radii = {0.02, 0.03, 0.04};
        auto mesh = geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*o3d_pcd, radii);

        mesh->RemoveDuplicatedVertices();
        mesh->RemoveDuplicatedTriangles();
        mesh->ComputeVertexNormals();

        // Save PLY
        io::WriteTriangleMesh(output_ply_path, *mesh);

        helper_functions::MeshResult result;
        result.vertices.reserve(mesh->vertices_.size());
        for (const auto &v : mesh->vertices_) result.vertices.push_back(v);
        result.triangles.reserve(mesh->triangles_.size());
        for (const auto &t : mesh->triangles_) result.triangles.push_back({t(0), t(1), t(2)});
        return result;
    }

}
