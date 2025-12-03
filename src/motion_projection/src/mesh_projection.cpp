#include "motion_projection/mesh_projection.hpp"

#include "reconstruction/ReconState.hpp"

#include <queue>
#include <limits>
#include <iostream>

namespace reconstruction {

// -------------------------------------------------------------
// 1. Ray–triangle intersection (Möller–Trumbore)
// -------------------------------------------------------------
bool MeshProjection::rayTriangleIntersect(
        const Eigen::Vector3d& orig,
        const Eigen::Vector3d& dir,
        const Eigen::Vector3d& v0,
        const Eigen::Vector3d& v1,
        const Eigen::Vector3d& v2,
        double& t, double& u, double& v)
{
    constexpr double EPS = 1e-9;

    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;

    Eigen::Vector3d pvec = dir.cross(edge2);
    double det = edge1.dot(pvec);

    if (std::abs(det) < EPS) return false;
    double invDet = 1.0 / det;

    Eigen::Vector3d tvec = orig - v0;
    u = tvec.dot(pvec) * invDet;
    if (u < 0.0 || u > 1.0) return false;

    Eigen::Vector3d qvec = tvec.cross(edge1);
    v = dir.dot(qvec) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;

    t = edge2.dot(qvec) * invDet;
    if (t <= EPS) return false;

    return true;
}

// -------------------------------------------------------------
// 2. Find triangle containing a point
// -------------------------------------------------------------
int MeshProjection::findContainingTriangle(const ReconState& recon,
                                           const Eigen::Vector3d& point,
                                           double planeEps,
                                           double baryEps)
{
    if (!recon.mesh_) return -1;
    const auto& tris = recon.mesh_->triangles_;
    const auto& verts = recon.mesh_->vertices_;

    for (int t = 0; t < (int)tris.size(); ++t) {
        const Eigen::Vector3i& tri = tris[t];
        const Eigen::Vector3d& p0 = verts[tri[0]];
        const Eigen::Vector3d& p1 = verts[tri[1]];
        const Eigen::Vector3d& p2 = verts[tri[2]];

        Eigen::Vector3d v0 = p1 - p0;
        Eigen::Vector3d v1 = p2 - p0;
        Eigen::Vector3d v2 = point - p0;

        Eigen::Vector3d n = v0.cross(v1);
        if (n.norm() < 1e-12) continue;
        n.normalize();

        if (std::abs((point - p0).dot(n)) > planeEps) continue;

        // barycentric test
        double d00 = v0.dot(v0);
        double d01 = v0.dot(v1);
        double d11 = v1.dot(v1);
        double d20 = v2.dot(v0);
        double d21 = v2.dot(v1);

        double denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < 1e-15) continue;

        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1 - v - w;

        if (u >= -baryEps && v >= -baryEps && w >= -baryEps &&
            u <= 1 + baryEps && v <= 1 + baryEps && w <= 1 + baryEps)
            return t;
    }

    return -1;
}

// -------------------------------------------------------------
// 3. Collect triangle neighborhood (k rings)
// -------------------------------------------------------------
std::vector<int> MeshProjection::collectTriangleNeighborhood(
        const ReconState& recon,
        int startTri,
        int maxRings)
{
    std::vector<int> out;

    if (!recon.mesh_) return out;
    if (recon.vertexTriAdjacencyMap_.empty()) return out;

    const size_t triCount = recon.mesh_->triangles_.size();
    if (startTri < 0 || startTri >= (int)triCount) return out;

    std::vector<bool> visited(triCount, false);
    std::queue<std::pair<int,int>> q;

    visited[startTri] = true;
    q.push({startTri, 0});
    out.push_back(startTri);

    while (!q.empty()) {
        auto [tri, depth] = q.front();
        q.pop();

        if (depth >= maxRings) continue;

        const Eigen::Vector3i& t = recon.mesh_->triangles_[tri];

        for (int k = 0; k < 3; k++) {
            int v = t[k];
            for (int nb : recon.vertexTriAdjacencyMap_[v]) {
                if (!visited[nb]) {
                    visited[nb] = true;
                    q.push({nb, depth + 1});
                    out.push_back(nb);
                }
            }
        }
    }

    return out;
}

// -------------------------------------------------------------
// 4. Main projection algorithm
// -------------------------------------------------------------
LocalProjectionResult MeshProjection::projectPointToMeshLocal(
        const ReconState& recon,
        const Eigen::Vector3d& startPoint,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& direction,
        int startTriangle,
        int maxRings,
        double maxDistance)
{
    LocalProjectionResult result;

    if (!recon.mesh_) return result;
    if (recon.mesh_->triangles_.empty()) return result;
    if (recon.vertexTriAdjacencyMap_.empty()) {
        std::cerr << "Call recon.computeVertexTriAdjacencyMap() first!\n";
        return result;
    }

    // 1) Compute new point
    Eigen::Vector3d newPoint = startPoint + velocity;

    Eigen::Vector3d rayOrig = newPoint;
    Eigen::Vector3d rayDir  = direction.normalized();

    if (maxDistance <= 0)
        maxDistance = velocity.norm() * 2.0;

    // 2) Collect local triangle neighborhood
    auto candidates = collectTriangleNeighborhood(recon, startTriangle, maxRings);
    if (candidates.empty()) return result;

    // 3) Ray–triangle test
    double bestT = std::numeric_limits<double>::infinity();
    int bestTri = -1;
    Eigen::Vector3d bestPoint;

    for (int triIdx : candidates) {
        const auto& tri = recon.mesh_->triangles_[triIdx];

        double t, u, v;
        if (rayTriangleIntersect(rayOrig, rayDir,
                recon.mesh_->vertices_[tri[0]],
                recon.mesh_->vertices_[tri[1]],
                recon.mesh_->vertices_[tri[2]],
                t, u, v))
        {
            if (t > 0 && t < bestT && t < maxDistance) {
                bestT = t;
                bestTri = triIdx;
                bestPoint = rayOrig + t * rayDir;
            }
        }
    }

    if (bestTri == -1)
        return result;

    result.hit = true;
    result.triangleIndex = bestTri;
    result.point = bestPoint;
    result.distance = bestT;
    return result;
}

} // namespace reconstruction
