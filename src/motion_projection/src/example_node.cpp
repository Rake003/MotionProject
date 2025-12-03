#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "motion_projection/ros_kinematics_utility.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string urdf_path = "src/Universal_Robots_ROS2_Description/urdf/ur5e.urdf";

    std::vector<std::string> joint_names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    auto node = std::make_shared<RosKinematicsUtility>(
        urdf_path,
        joint_names,
        "base_link",
        "tool0",
        "Flange"
    );

    // Test FK
    Eigen::VectorXd q(6);
    q << 0, -0.5, 0.5, 0, 0.3, 0;
    auto fk = node->kdl_model_.computeFK(q, false, "Flange");
    node->moveJointSpace(q);
    std::cout << "FK rotation vector: " << fk.transpose() << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
