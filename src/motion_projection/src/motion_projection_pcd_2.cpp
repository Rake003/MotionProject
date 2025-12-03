#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "motion_projection/ros_kinematics_utility.hpp"
#include "motion_projection/helper_functions.hpp"
#include "reconstruction/ReconState.hpp"
#include "motion_projection/mesh_projection.hpp"
#include "motion_projection/mesh_visualization.hpp"

using namespace helper_functions;
using namespace reconstruction;
using namespace motion_projection;

// =========================================================
// Non-blocking keyboard input
// =========================================================
char getKey()
{
    char c = 0;
    int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
    read(STDIN_FILENO, &c, 1);
    fcntl(STDIN_FILENO, F_SETFL, old_flags);
    return c;
}

Eigen::VectorXd getKeyboardVelocity()
{
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);
    double velocity_level = 0.005;

    char k = getKey();
    switch (k)
    {
        case 'w': vel[2] =  velocity_level; break;  // +Z
        case 's': vel[2] = -velocity_level; break;  // -Z
        case 'a': vel[1] =  velocity_level; break;  // +Y
        case 'd': vel[1] = -velocity_level; break;  // -Y
        case 'q': vel[0] =  velocity_level; break;  // +X
        case 'e': vel[0] = -velocity_level; break;  // -X
        default: break;
    }
    return vel;
}

// Helper: rotation vector to matrix using Rodrigues formula
Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d& rotvec)
{
    double angle = rotvec.norm();
    if (angle < 1e-10) return Eigen::Matrix3d::Identity();
    
    Eigen::Vector3d axis = rotvec.normalized();
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

// Helper: rotation matrix to vector using Eigen's AngleAxis
Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

// Helper: Build rotation matrix from Z-axis direction
Eigen::Matrix3d buildRotationFromZ(const Eigen::Vector3d& z_dir)
{
    Eigen::Vector3d z = z_dir.normalized();
    
    // Find a vector not parallel to z
    Eigen::Vector3d temp;
    if (std::abs(z.x()) < 0.9) {
        temp = Eigen::Vector3d(1, 0, 0);
    } else {
        temp = Eigen::Vector3d(0, 1, 0);
    }
    
    // Build orthonormal frame
    Eigen::Vector3d x = temp.cross(z).normalized();
    Eigen::Vector3d y = z.cross(x).normalized();
    
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    
    return R;
}

// Helper: Compute angular velocity to rotate from R_current to R_target
Eigen::Vector3d computeAngularVelocity(const Eigen::Matrix3d& R_current, 
                                        const Eigen::Matrix3d& R_target, 
                                        double dt)
{
    // Compute rotation error: R_error = R_target * R_current^T
    Eigen::Matrix3d R_error = R_target * R_current.transpose();
    
    // Convert to axis-angle
    Eigen::AngleAxisd aa(R_error);
    double angle = aa.angle();
    Eigen::Vector3d axis = aa.axis();
    
    // Normalize angle to [-pi, pi]
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    
    // Angular velocity in world frame
    Eigen::Vector3d omega = (angle / dt) * axis;
    
    return omega;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    ReconState recon;

    std::string urdf_path = "src/Universal_Robots_ROS2_Description/urdf/ur5e.urdf";
    std::vector<std::string> joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    auto robot = std::make_shared<RosKinematicsUtility>(
        urdf_path, joint_names, "base_link", "tool0", "Flange");

    Eigen::VectorXd init_joint_angle(6), init_cart_pose(6);

    double dt = 0.010;
    rclcpp::Rate loop_rate(1.0 / dt);

    init_joint_angle << 0, -1.57, 1.57, 1.57, 0, 0;
    robot->moveJointSpace(init_joint_angle);

    // =========================================================
    // Generate sphere
    // =========================================================
    double r = 0.100;
    Eigen::Vector3d center(0.4, 0, 0);
    auto pcd = generate_sphere_pcd(r, center, 2000);

    std::cout << "Generated " << pcd.size() << " points for sphere.\n";
    std::cout << "Sphere center: " << center.transpose() << ", radius: " << r << "\n";

    // =========================================================
    // Triangulate
    // =========================================================
    std::string save_path = "sphere_mesh.ply";
    MeshResult result = triangulate_with_open3d(pcd, save_path);

    if (!recon.readMeshFromFile(save_path)) 
    {
        std::cerr << "Failed to read mesh.\n";
        return -1;
    }
    std::cout << "Mesh loaded: " 
              << recon.mesh_->vertices_.size() << " vertices, "
              << recon.mesh_->triangles_.size() << " triangles.\n";

    recon.computeVertexTriAdjacencyMap();
    if (!recon.isMeshVertexTriAdjacencyMatched()) 
    {
        std::cerr << "Vertex adjacency failed.\n";
        return -1;
    }

    recon.mesh_->ComputeTriangleNormals();
    std::cout << "Triangle normals computed.\n";

    // =========================================================
    // Initial position: TOP of sphere
    // =========================================================
    Eigen::Vector3d target_point = center + Eigen::Vector3d(0, 0, r);
    
    std::cout << "Initial target (top of sphere): " << target_point.transpose() << "\n";

    int current_tri = MeshProjection::findContainingTriangle(recon, target_point);
    if (current_tri < 0) 
    {
        std::cout << "Exact top not found, searching nearest...\n";
        
        double min_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < recon.mesh_->triangles_.size(); ++i) {
            const auto& tri = recon.mesh_->triangles_[i];
            Eigen::Vector3d tri_center = (
                recon.mesh_->vertices_[tri[0]] +
                recon.mesh_->vertices_[tri[1]] +
                recon.mesh_->vertices_[tri[2]]
            ) / 3.0;
            
            double dist = (tri_center - target_point).norm();
            if (dist < min_dist) {
                min_dist = dist;
                current_tri = i;
            }
        }
        
        if (current_tri >= 0) {
            const auto& tri = recon.mesh_->triangles_[current_tri];
            target_point = (
                recon.mesh_->vertices_[tri[0]] +
                recon.mesh_->vertices_[tri[1]] +
                recon.mesh_->vertices_[tri[2]]
            ) / 3.0;
            std::cout << "Found triangle " << current_tri << " at distance " << min_dist << "\n";
        }
    }
    
    if (current_tri < 0) 
    {
        std::cerr << "Could not find containing triangle.\n";
        return -1;
    }
    
    std::cout << "Start triangle: " << current_tri << "\n";
    std::cout << "Adjusted target: " << target_point.transpose() << "\n";

    Eigen::Vector3d triNormal = recon.mesh_->triangle_normals_[current_tri];
    std::cout << "Initial normal: " << triNormal.transpose() << "\n";

    // =========================================================
    // INITIAL ORIENTATION
    // =========================================================
    
    // Determine outward normal direction
    Eigen::Vector3d center_to_point = (target_point - center).normalized();
    double dot_product = triNormal.dot(center_to_point);
    
    std::cout << "Center to point (radial): " << center_to_point.transpose() << "\n";
    
    // Get outward-pointing normal
    Eigen::Vector3d outward_normal;
    if (dot_product > 0) {
        outward_normal = triNormal.normalized();
    } else {
        outward_normal = -triNormal.normalized();
    }
    
    // Tool Z should point INWARD
    Eigen::Vector3d desired_tool_z = -outward_normal;
    
    std::cout << "Outward normal: " << outward_normal.transpose() << "\n";
    std::cout << "Desired tool Z (inward): " << desired_tool_z.transpose() << "\n";
    
    // Build rotation matrix
    Eigen::Matrix3d R_initial = buildRotationFromZ(desired_tool_z);
    Eigen::Vector3d rotvec_initial = rotationMatrixToVector(R_initial);
    
    std::cout << "Initial rotation matrix:\n" << R_initial << "\n";
    std::cout << "Initial rotvec: " << rotvec_initial.transpose() << "\n";
    std::cout << "Rotvec norm: " << rotvec_initial.norm() << " rad\n";
    
    // Check for invalid values
    if (!std::isfinite(rotvec_initial.norm()) || rotvec_initial.norm() > 10.0) {
        std::cerr << "ERROR: Invalid rotation vector computed!\n";
        std::cerr << "Using zero rotation instead.\n";
        rotvec_initial.setZero();
    }
    
    init_cart_pose << target_point.x(), target_point.y(), target_point.z(),
                      rotvec_initial.x(), rotvec_initial.y(), rotvec_initial.z();
    
    std::cout << "\nMoving to initial pose...\n";
    std::cout << "Press Enter to continue...\n";
    std::cin.get();
    
    robot->moveCartesianSpace(init_cart_pose);

    std::cout << "Press Enter to start control loop...\n";
    std::cin.get();
    
    // Settle
    for (int i = 0; i < 50; ++i) {
        rclcpp::spin_some(robot);
        loop_rate.sleep();
    }

    // =========================================================
    // MAIN LOOP - Use velocity control only
    // =========================================================
    std::cout << "\n=== Starting velocity control loop ===\n";
    std::cout << "Using velocity-based orientation control\n";
    std::cout << "Controls: WASD/QE to move, Ctrl+C to quit\n\n";
    
    int frame_count = 0;
    std::vector<int> last_candidates;
    
    while (rclcpp::ok())
    {
        frame_count++;
        
        // Publish point cloud
        robot->publishPointCloud(pcd, "base_link", robot->pcd1_pub_);

        // Get state
        Eigen::VectorXd q = robot->getCurrentJointPositions();
        Eigen::VectorXd pose = robot->getCurrentCartesianPose();
        Eigen::Vector3d current_pos = pose.head<3>();
        Eigen::Matrix3d R_current = rotationVectorToMatrix(pose.tail<3>());

        // User input
        Eigen::VectorXd user_vel = getKeyboardVelocity();
        
        if (user_vel.head<3>().norm() < 1e-6) {
            // Visualize even when not moving
            if (frame_count % 10 == 0) {
                publishMeshVisualization(recon, "base_link", robot->mesh_pub_, robot.get());
                publishSelectedTrianglesVisualization(recon, last_candidates, current_tri, 
                                                     "base_link", robot->normals_pub_, robot.get());
                publishCurrentPositionMarker(current_pos, "base_link", robot->trajectory_pub_, robot.get());
            }
            
            rclcpp::spin_some(robot);
            loop_rate.sleep();
            continue;
        }

        // Desired position from input
        Eigen::Vector3d desired_new_pos = current_pos + user_vel.head<3>();
        
        // =====================================================
        // Projection onto mesh
        // =====================================================
        
        // Use radial direction from sphere center
        Eigen::Vector3d ray_dir = (desired_new_pos - center).normalized();
        double offset = 0.05;
        Eigen::Vector3d ray_origin = center + (r - offset) * ray_dir;
        
        last_candidates = MeshProjection::collectTriangleNeighborhood(recon, current_tri, 15);
        
        auto proj_result = MeshProjection::projectPointToMeshLocal(
            recon, 
            ray_origin,
            Eigen::Vector3d::Zero(),
            ray_dir,
            current_tri, 
            15,
            0.2
        );
        
        if (!proj_result.hit)
        {
            last_candidates = MeshProjection::collectTriangleNeighborhood(recon, current_tri, 30);
            
            proj_result = MeshProjection::projectPointToMeshLocal(
                recon, 
                ray_origin,
                Eigen::Vector3d::Zero(),
                ray_dir,
                current_tri, 
                30,
                0.5
            );
            
            if (!proj_result.hit) {
                std::cout << "Frame " << frame_count << ": projection failed\n";
                rclcpp::spin_some(robot);
                loop_rate.sleep();
                continue;
            }
        }
        
        current_tri = proj_result.triangleIndex;
        Eigen::Vector3d new_normal = recon.mesh_->triangle_normals_[current_tri];
        
        // =====================================================
        // Compute target orientation
        // =====================================================
        
        Eigen::Vector3d center_to_new_point = (proj_result.point - center).normalized();
        double new_dot = new_normal.dot(center_to_new_point);
        
        Eigen::Vector3d outward_normal_new;
        if (new_dot > 0) {
            outward_normal_new = new_normal.normalized();
        } else {
            outward_normal_new = -new_normal.normalized();
        }
        
        // Target tool Z: inward
        Eigen::Vector3d target_tool_z = -outward_normal_new;
        
        // Build target rotation
        Eigen::Matrix3d R_target = buildRotationFromZ(target_tool_z);
        
        // =====================================================
        // Compute velocities (position and orientation)
        // =====================================================
        
        // Linear velocity
        Eigen::Vector3d position_error = proj_result.point - current_pos;
        Eigen::Vector3d linear_vel = position_error / dt;
        
        // Angular velocity
        Eigen::Vector3d angular_vel = computeAngularVelocity(R_current, R_target, dt);
        
        // Limit velocities
        double max_linear_vel = 0.1;
        double max_angular_vel = 1.0;
        
        if (linear_vel.norm() > max_linear_vel) {
            linear_vel = linear_vel.normalized() * max_linear_vel;
        }
        
        if (angular_vel.norm() > max_angular_vel) {
            angular_vel = angular_vel.normalized() * max_angular_vel;
        }
        
        // =====================================================
        // Joint velocity via Jacobian
        // =====================================================
        
        Eigen::VectorXd cmd_vel(6);
        cmd_vel << linear_vel, angular_vel;
        
        Eigen::MatrixXd J = robot->kdl_model_.computeJacobian(q, "Flange");
        
        double lambda = 0.01;
        Eigen::MatrixXd J_inv = (J.transpose() * J + lambda * Eigen::MatrixXd::Identity(6,6))
                                .ldlt().solve(J.transpose());
        
        Eigen::VectorXd q_dot = J_inv * cmd_vel;
        Eigen::VectorXd q_new = q + dt * q_dot;
        
        robot->moveJointSpace(q_new);
        
        // =====================================================
        // VISUALIZATION
        // =====================================================
        if (frame_count % 5 == 0) {
            publishMeshVisualization(recon, "base_link", robot->mesh_pub_, robot.get());
            publishSelectedTrianglesVisualization(recon, last_candidates, current_tri, 
                                                 "base_link", robot->normals_pub_, robot.get());
            publishCurrentPositionMarker(current_pos, "base_link", robot->trajectory_pub_, robot.get());
        }
        
        // Debug output
        if (frame_count % 50 == 0) {
            std::cout << "\n=== Frame " << frame_count << " ===\n";
            std::cout << "Triangle: " << current_tri << "\n";
            std::cout << "Position: " << proj_result.point.transpose() << "\n";
            std::cout << "Outward normal: " << outward_normal_new.transpose() << "\n";
            std::cout << "Target tool Z (inward): " << target_tool_z.transpose() << "\n";
            std::cout << "Current tool Z: " << R_current.col(2).transpose() << "\n";
            std::cout << "Alignment: " << R_current.col(2).dot(target_tool_z) << " (want ~1)\n";
            std::cout << "Angular vel: " << angular_vel.transpose() << " rad/s\n";
            std::cout << "Angular vel norm: " << angular_vel.norm() << " rad/s\n\n";
        }

        rclcpp::spin_some(robot);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}