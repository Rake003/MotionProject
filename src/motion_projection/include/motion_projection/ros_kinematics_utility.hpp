#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Your FK/IK/Jacobian model
#include <kdl_robot_model.hpp>

/**
 * @brief Generic Kinematics Utility (Rotation Vector Only)
 *
 * Pose Format: [x, y, z, rx, ry, rz]
 * where (rx, ry, rz) is rotation vector = axis * angle (radians)
 */
class RosKinematicsUtility : public rclcpp::Node
{
    public:
        RosKinematicsUtility(const std::string &urdf_path,
                            const std::vector<std::string> &joint_names,
                            const std::string &base_frame,
                            const std::string &tip_frame,
                            const std::string &computation_tip_frame);

        void run();

        void moveJointSpace(const Eigen::VectorXd &joint_angles_rad);
        void moveCartesianSpace(const Eigen::VectorXd &target_pose);

        void publishJointState(const std::vector<double> &joint_positions);
        void publishPointCloud(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);

        Eigen::VectorXd getCurrentJointPositions() const;
        Eigen::VectorXd getCurrentCartesianPose() const;

    public:
        KDLRobotModel kdl_model_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_; // publish joint angles
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd1_pub_; // publish point cloud 1
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd2_pub_; // publish point cloud 2
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normals_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;

    private:
        bool validateJointVectorSize(const Eigen::VectorXd &q) const;

        std::string base_frame_;
        std::string tip_frame_;

        std::vector<std::string> joint_names_;
        size_t dof_;

        Eigen::VectorXd current_joint_positions_;
        Eigen::VectorXd current_cartesian_pose_;
};
