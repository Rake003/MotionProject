/**
 * @file flexiv_ros_kinematics_utils.hpp
 * @author Rakesh Kumar K (rakeshkumar.k@htic.iitm.ac.in)
 * @brief Header file for flexiv ROS kinematics utility node
 * @version 0.1
 * @date 2025-11-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "kdl_robot_model.hpp"


class FlexivROSKinematicsUtils : public rclcpp::Node
{
public:
    KDLRobotModel kdl_model_flexiv;
    explicit FlexivROSKinematicsUtils(const std::string &urdf_path);

    void run();   // main interactive loop

    void moveJointSpace(Eigen::VectorXd &joint_angles_rad);
    void moveCartesianSpace(Eigen::VectorXd &target_pose);
    void publishJointState(const std::vector<double> &joint_positions);
    Eigen::VectorXd getCurrentJointPositions();
    Eigen::VectorXd getCurrentCartesianPose();

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::vector<std::string> joint_names_;

    // Store current state
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd current_cartesian_pose_;
};
