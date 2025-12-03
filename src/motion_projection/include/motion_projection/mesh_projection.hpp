#pragma once

#include <Eigen/Dense>
#include <vector>

namespace reconstruction {

class ReconState; // forward declaration

struct LocalProjectionResult {
    bool hit = false;
    int  triangleIndex = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double distance = std::numeric_limits<double>::infinity();
};

class MeshProjection {
public:
    // Find triangle that contains a 3D point (slow global search)
    static int findContainingTriangle(const ReconState& recon,
                                      const Eigen::Vector3d& point,
                                      double planeEps = 1e-5,
                                      double baryEps  = 1e-5);

    // Collect triangles in N "rings" around a starting triangle
    static std::vector<int> collectTriangleNeighborhood(
            const ReconState& recon,
            int startTri,
            int maxRings);

    // Main algorithm:
    // move startPoint by velocity → cast ray → project back to mesh
    static LocalProjectionResult projectPointToMeshLocal(
            const ReconState& recon,
            const Eigen::Vector3d& startPoint,
            const Eigen::Vector3d& velocity,
            const Eigen::Vector3d& direction,
            int startTriangle,
            int maxRings = 10,
            double maxDistance = -1.0);

private:
    // Möller–Trumbore ray-triangle intersection
    static bool rayTriangleIntersect(
            const Eigen::Vector3d& orig,
            const Eigen::Vector3d& dir,
            const Eigen::Vector3d& v0,
            const Eigen::Vector3d& v1,
            const Eigen::Vector3d& v2,
            double& t, double& u, double& v);
};

} // namespace reconstruction
