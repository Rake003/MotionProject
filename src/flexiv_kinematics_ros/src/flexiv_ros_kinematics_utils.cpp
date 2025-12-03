/**
 * @file flexiv_ros_kinematics_utils.cpp
 * @author Rakesh Kumar K (rakeshkumar.k@htic.iitm.ac.in)
 * @brief Source file for flexiv ROS kinematics utility node. It contains methods that can be used
 *        to perform forward and inverse kinematics as well as publish joint states to ROS topic.
 * @version 0.1
 * @date 2025-11-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "flexiv_kinematics_ros/flexiv_ros_kinematics_utils.hpp"
#include <iostream>
#include <sstream>
#include <thread>
#include <cmath>

FlexivROSKinematicsUtils::FlexivROSKinematicsUtils(const std::string &urdf_path)
    : Node("flexiv_move_node"), 
      kdl_model_flexiv(urdf_path, "base_link", "flange")
{
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
                    "joint_states", qos);

    joint_names_ = {
        "Rizon4s-000000_joint1",
        "Rizon4s-000000_joint2",
        "Rizon4s-000000_joint3",
        "Rizon4s-000000_joint4",
        "Rizon4s-000000_joint5",
        "Rizon4s-000000_joint6",
        "Rizon4s-000000_joint7"
    };

    // Initialize to home position (all zeros)
    current_joint_positions_ = Eigen::VectorXd::Zero(7);
    current_cartesian_pose_ = Eigen::VectorXd::Zero(6);
    
    // Compute initial FK to set cartesian pose
    current_cartesian_pose_ = kdl_model_flexiv.computeFK(current_joint_positions_, true, "Flange");

    RCLCPP_INFO(this->get_logger(), "Flexiv Move Node initialized");
}

void FlexivROSKinematicsUtils::run()
{
    std::string mode;

    while (rclcpp::ok())
    {
        std::cout << "\n=== Flexiv Robot Control ===\n";
        std::cout << "  1 = Joint Space Control\n";
        std::cout << "  2 = Cartesian Space Control (IK)\n";
        std::cout << "  3 = Show Current Joint Positions\n";
        std::cout << "  4 = Show Current Cartesian Pose\n";
        std::cout << "  q = Quit\n";
        std::cout << "Enter choice: ";

        if (!std::getline(std::cin, mode)) break;

        if (mode == "1")
        {
            std::cout << "\nEnter 7 joint angles (deg): ";
            std::string input; 
            std::getline(std::cin, input);

            std::istringstream iss(input);
            std::vector<double> vals;
            double a;
            while (iss >> a) vals.push_back(a);

            if (vals.size() != 7)
            {
                RCLCPP_ERROR(this->get_logger(), "Need 7 values.");
                continue;
            }

            Eigen::VectorXd q(7);
            for (int i = 0; i < 7; i++) q[i] = vals[i] * M_PI / 180.0;

            moveJointSpace(q);
        }
        else if (mode == "2")
        {
            std::cout << "\nEnter x y z roll pitch yaw (deg): ";
            std::string input; 
            std::getline(std::cin, input);

            std::istringstream iss(input);
            std::vector<double> vals;
            double v;
            while (iss >> v) vals.push_back(v);

            if (vals.size() != 6)
            {
                RCLCPP_ERROR(this->get_logger(), "Need 6 values.");
                continue;
            }

            Eigen::VectorXd pose(6);
            pose << vals[0], vals[1], vals[2],
                    vals[3]*M_PI/180, vals[4]*M_PI/180, vals[5]*M_PI/180;

            moveCartesianSpace(pose);
        }
        else if (mode == "3")
        {
            Eigen::VectorXd joints = getCurrentJointPositions();
            std::cout << "\nCurrent Joint Positions (rad):\n";
            for (int i = 0; i < joints.size(); i++)
            {
                std::cout << "  Joint " << (i+1) << ": " << joints[i] 
                          << " rad (" << joints[i] * 180.0 / M_PI << " deg)\n";
            }
        }
        else if (mode == "4")
        {
            Eigen::VectorXd pose = getCurrentCartesianPose();
            if (pose.size() >= 6)
            {
                std::cout << "\nCurrent Cartesian Pose:\n";
                std::cout << "  Position (x, y, z): " 
                          << pose[0] << ", " << pose[1] << ", " << pose[2] << " m\n";
                std::cout << "  Orientation (roll, pitch, yaw): "
                          << pose[3] * 180.0 / M_PI << ", "
                          << pose[4] * 180.0 / M_PI << ", "
                          << pose[5] * 180.0 / M_PI << " deg\n";
            }
        }
        else if (mode == "q" || mode == "Q")
        {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            break;
        }
        else
        {
            std::cout << "Invalid choice.\n";
        }
    }
}

void FlexivROSKinematicsUtils::moveJointSpace(Eigen::VectorXd &q)
{
    // Compute FK for display
    Eigen::VectorXd fk = kdl_model_flexiv.computeFK(q, true, "Flange");

    if (fk.size() >= 6)
    {
        std::cout << "\nFK Result:\n";
        std::cout << "  Position (x, y, z): " 
                  << fk[0] << ", " << fk[1] << ", " << fk[2] << " m\n";
        std::cout << "  Orientation (roll, pitch, yaw): "
                  << fk[3] * 180.0 / M_PI << ", "
                  << fk[4] * 180.0 / M_PI << ", "
                  << fk[5] * 180.0 / M_PI << " deg\n";
    }

    // Publish joint state (this will update current state)
    std::vector<double> joints(7);
    for (int i = 0; i < 7; i++) joints[i] = q[i];

    publishJointState(joints);
}

void FlexivROSKinematicsUtils::moveCartesianSpace(Eigen::VectorXd &target_pose)
{
    // Compute IK
    Eigen::VectorXd ik = kdl_model_flexiv.computeIK(target_pose, true, "Flange");

    if (ik.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "IK failed.");
        return;
    }

    std::cout << "\nIK Solution (rad): " << ik.transpose() << "\n";
    std::cout << "IK Solution (deg): ";
    for (int i = 0; i < ik.size(); i++)
    {
        std::cout << ik[i] * 180.0 / M_PI << " ";
    }
    std::cout << "\n";

    // Publish joint state (this will update current state)
    std::vector<double> joints(ik.data(), ik.data() + ik.size());
    if (joints.size() < joint_names_.size())
        joints.resize(joint_names_.size(), 0.0);

    publishJointState(joints);
}

void FlexivROSKinematicsUtils::publishJointState(const std::vector<double> &pos)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = pos;

    if (msg.position.size() != msg.name.size())
        msg.position.resize(msg.name.size(), 0.0);

    joint_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published joint state");
    
    // Update current joint positions from published values
    current_joint_positions_.resize(pos.size());
    for (size_t i = 0; i < pos.size(); i++)
    {
        current_joint_positions_[i] = pos[i];
    }
    
    // Update current Cartesian pose using FK
    current_cartesian_pose_ = kdl_model_flexiv.computeFK(current_joint_positions_, true, "Flange");
}

Eigen::VectorXd FlexivROSKinematicsUtils::getCurrentJointPositions()
{
    return current_joint_positions_;
}

Eigen::VectorXd FlexivROSKinematicsUtils::getCurrentCartesianPose()
{
    return current_cartesian_pose_;
}