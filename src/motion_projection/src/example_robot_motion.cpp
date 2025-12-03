#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "motion_projection/ros_kinematics_utility.hpp"
#include "motion_projection/helper_functions.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string urdf_path = "src/Universal_Robots_ROS2_Description/urdf/ur5e.urdf";
    std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    auto robot = std::make_shared<RosKinematicsUtility>(urdf_path,joint_names,"base_link","tool0", "Flange");

    Eigen::VectorXd init_joint_angle(6), init_pose(6), current_joint_angle(6), current_pose(6), vel(6);

    // 1 kHz loop update
    double dt = 0.010;
    rclcpp::Rate loop_rate(1.0 / dt);   // 1000 Hz

    init_joint_angle << 0, -1.57, 1.57, 1.57, 0, 0;
    robot->moveJointSpace(init_joint_angle);

    init_joint_angle = robot->getCurrentJointPositions();
    init_pose = robot->getCurrentCartesianPose();

    double r = 0.100;
    Eigen::Vector3d center(0.4,0,0);
    auto pcd = helper_functions::generate_sphere_pcd(r, center, 2000);
    // auto pcd_point_cloud = helper_functions::eigenToPointCloud2(pcd);

    for (auto& p : pcd)
        std::cout << p.transpose() << std::endl;

    // Main loops
    while (rclcpp::ok())
    {
        // Publish the sphere cloud every loop
        robot->publishPointCloud(pcd, "base_link", robot->pcd1_pub_);

        // Get current state
        current_joint_angle = robot->getCurrentJointPositions();
        current_pose        = robot->getCurrentCartesianPose();

        // give some velocity
        vel << 0, 0, 0.001, 0, 0, 0;
        Eigen::MatrixXd Jacobian = robot->kdl_model_.computeJacobian(current_joint_angle, "Flange");

        Eigen::VectorXd joint_vel = Eigen::Inverse(Jacobian) * vel;

        joint_vel = joint_vel * dt;
        Eigen::VectorXd new_joint_angle = current_joint_angle + joint_vel;

        robot->moveJointSpace(new_joint_angle);

        // Allow ROS callbacks
        rclcpp::spin_some(robot);

        // Sleep to maintain loop timing
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
