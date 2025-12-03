#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <termios.h>      // for keyboard input
#include <unistd.h>
#include <fcntl.h>

#include "motion_projection/ros_kinematics_utility.hpp"
#include "motion_projection/helper_functions.hpp"

// =========================================================
// Non-blocking keyboard input (WASD + QE for XYZ velocity)
// =========================================================
char getKey()
{
    char c = 0;

    // make terminal non-blocking
    int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

    read(STDIN_FILENO, &c, 1);

    // restore flags
    fcntl(STDIN_FILENO, F_SETFL, old_flags);

    return c;
}

Eigen::VectorXd getKeyboardVelocity()
{
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);

    double velocity_level_plus  = 0.010;
    double velocity_level_minus = -0.010;

    char k = getKey();

    switch (k)
    {
        case 'w': vel[2] = velocity_level_plus;  break;  // +Z
        case 's': vel[2] = velocity_level_minus; break;  // -Z
        case 'a': vel[1] = velocity_level_plus;  break;  // +Y
        case 'd': vel[1] = velocity_level_minus; break;  // -Y
        case 'q': vel[0] = velocity_level_plus;  break;  // +X
        case 'e': vel[0] = velocity_level_minus; break;  // -X
        default: break;
    }

    return vel;
}

// =========================================================
// MAIN
// =========================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string urdf_path = "src/Universal_Robots_ROS2_Description/urdf/ur5e.urdf";
    std::vector<std::string> joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    auto robot = std::make_shared<RosKinematicsUtility>(
        urdf_path, joint_names, "base_link", "tool0", "Flange");

    Eigen::VectorXd init_joint_angle(6), init_cart_pose(6);

    // 100 Hz control loop
    double dt = 0.010;
    rclcpp::Rate loop_rate(1.0 / dt);

    // Move robot to a default useful pose
    init_joint_angle << 0, -1.57, 1.57, 1.57, 0, 0;
    robot->moveJointSpace(init_joint_angle);

    // =========================================================
    // Generate sphere PCD
    // =========================================================
    double r = 0.100;
    Eigen::Vector3d center(0.4, 0, 0);
    auto pcd = helper_functions::generate_sphere_pcd(r, center, 2000);

    // =========================================================
    // Do triangulation for the given pcd points (along with normals with some data structure)
    // =========================================================

    // =========================================================
    // Pick a random point with z > 0
    // =========================================================
    Eigen::Vector3d target_point = center;
    for (int i = 0; i < 2000; ++i)
    {
        if (pcd[i].z() > 0.0)
        {
            target_point = pcd[i];
            break;
        }
    }

    // Compute normal of the corresponding point

    std::cout << "Selected initial target point: "
              << target_point.transpose() << std::endl;

    // Build initial pose: keep original rotation, replace xyz
    init_cart_pose = robot->getCurrentCartesianPose();
    init_cart_pose[0] = target_point.x();
    init_cart_pose[1] = target_point.y();
    init_cart_pose[2] = target_point.z();
    // Change init orientation tooo

    // Move robot to that selected sphere point
    robot->moveCartesianSpace(init_cart_pose);

    // =========================================================
    // MAIN CONTROL LOOP
    // =========================================================
    while (rclcpp::ok())
    {
        // Publish sphere cloud
        robot->publishPointCloud(pcd, "base_link", robot->pcd1_pub_);

        // Get robot states
        Eigen::VectorXd q = robot->getCurrentJointPositions();
        Eigen::VectorXd pose = robot->getCurrentCartesianPose();

        // =====================================================
        // Velocity from keyboard
        // =====================================================
        Eigen::VectorXd vel = getKeyboardVelocity();  // XYZ linear vel

        // project that in negative z axis...find the corresponding projection on mesh...that point and normal
        // find this new velocity 

        // =====================================================
        // Compute joint velocity from Jacobian (DLS solver)
        // =====================================================
        Eigen::MatrixXd J = robot->kdl_model_.computeJacobian(q, "Flange");

        double lambda = 0.01;  // damping factor
        Eigen::MatrixXd J_inv =
            (J.transpose() * J + lambda * Eigen::MatrixXd::Identity(6,6)).ldlt().solve(J.transpose());

        Eigen::VectorXd q_dot = J_inv * vel;

        // Integrate
        Eigen::VectorXd q_new = q + dt * q_dot;

        // Move robot
        robot->moveJointSpace(q_new);

        rclcpp::spin_some(robot);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
