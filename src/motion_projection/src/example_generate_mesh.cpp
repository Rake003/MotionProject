#include <iostream>
#include <Eigen/Dense>
#include "motion_projection/helper_functions.hpp"

int main()
{
    using namespace helper_functions;

    // =============================================================
    // 1. Generate sphere point cloud
    // =============================================================
    double sphere_radius = 0.10;
    Eigen::Vector3d center(0.4, 0.0, 0.0);
    int num_points = 2000;

    std::vector<Eigen::Vector3d> pcd =
        generate_sphere_pcd(sphere_radius, center, num_points);

    std::cout << "Generated point cloud with " << pcd.size() << " points.\n";

    // =============================================================
    // 2. Triangulate using Open3D + save mesh
    // =============================================================
    std::string save_path = "sphere_mesh.ply";

    MeshResult result = triangulate_with_open3d(pcd, save_path);

    // =============================================================
    // 3. Print mesh stats
    // =============================================================
    std::cout << "Mesh vertices:  " << result.vertices.size() << std::endl;
    std::cout << "Mesh triangles: " << result.triangles.size() << std::endl;

    std::cout << "Mesh saved to " << save_path << "\n";

    return 0;
}
