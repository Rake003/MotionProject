#include <iostream>
#include <Eigen/Dense>

#include "reconstruction/ReconState.hpp"        // <-- full definition
#include "motion_projection/helper_functions.hpp"
#include "motion_projection/mesh_projection.hpp"

int main()
{
    using namespace helper_functions;
    using namespace reconstruction;

    ReconState recon;

    // =============================================================
    // Generate sphere point cloud
    // =============================================================
    double sphere_radius = 0.10;
    Eigen::Vector3d center(0.4, 0.0, 0.0);
    int num_points = 2000;

    std::vector<Eigen::Vector3d> pcd = generate_sphere_pcd(sphere_radius, center, num_points);

    std::cout << "Generated point cloud with " << pcd.size() << " points.\n";

    // =============================================================
    // Triangulate using Open3D + save mesh
    // =============================================================
    std::string save_path = "sphere_mesh.ply";

    MeshResult result = triangulate_with_open3d(pcd, save_path);

    if (!recon.readMeshFromFile(save_path)) 
    {
        std::cerr << "Failed to read mesh." << std::endl;
        return -1;
    }
    std::cout << "Mesh loaded successfully.\n";

    // ----------------------------------------------------------
    // Build vertex → triangle adjacency
    // ----------------------------------------------------------
    recon.computeVertexTriAdjacencyMap();

    if (!recon.isMeshVertexTriAdjacencyMatched()) 
    {
        std::cerr << "Vertex adjacency map computation failed.\n";
        return -1;
    }

    // compute normal for all the triangles
    recon.mesh_->ComputeTriangleNormals();

    // ======================================================================
    // EXAMPLE: Local mesh projection using MeshProjection
    // ======================================================================

    // 1) Select a vertex on the mesh as starting point
    Eigen::Vector3d startPoint = recon.mesh_->vertices_[0];
    std::cout << "Start point: " << startPoint.transpose() << "\n";

    // 2) Find which triangle this point lies in
    int tri = MeshProjection::findContainingTriangle(recon, startPoint);
    if (tri < 0) 
    {
        std::cerr << "Could not find containing triangle.\n";
        return -1;
    }
    std::cout << "Start point lies in triangle index: " << tri << "\n";

    // 3) Define a velocity (moves point outside mesh)
    Eigen::Vector3d vel(0.05, 0.02, -0.01);

    // 4) Define ray direction for projection back (e.g., inward)
    Eigen::Vector3d dir = -vel.normalized();   // cast backwards

    // 5) Perform local projection
    auto resultProj = MeshProjection::projectPointToMeshLocal(recon, startPoint, vel, dir,
                                                                                            tri,           // start triangle
                                                                                            10,            // search 10 adjacency rings
                                                                                            -1             // auto max distance
                                                                                        );
                                                
    int hitTri = resultProj.triangleIndex;
    Eigen::Vector3d triNormal = recon.mesh_->triangle_normals_[hitTri];

    // 6) Print result
    if (!resultProj.hit) 
    {
        std::cout << "Projection failed (point did not hit mesh locally)." << std::endl;
    } 
    else 
    {
        std::cout << "Projection HIT!" << std::endl;
        std::cout << "  Hit triangle index: " << resultProj.triangleIndex << std::endl;
        std::cout << "  Hit point: " << resultProj.point.transpose() << std::endl;
        std::cout << "  Distance along ray: " << resultProj.distance << std::endl;
        std::cout << "  Normal direction : " << triNormal << std::endl;
    }

    return 0;
}
