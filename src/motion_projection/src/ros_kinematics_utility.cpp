#include "motion_projection/ros_kinematics_utility.hpp"

RosKinematicsUtility::RosKinematicsUtility(const std::string &urdf_path,
                                           const std::vector<std::string> &joint_names,
                                           const std::string &base_frame,
                                           const std::string &tip_frame,
                                           const std::string &computation_tip_frame)
    : Node("kinematics_utility_node"),
      kdl_model_(urdf_path, base_frame, tip_frame),
      base_frame_(base_frame),
      tip_frame_(computation_tip_frame),
      joint_names_(joint_names),
      dof_(joint_names.size())
{
    if (dof_ == 0) {
        throw std::runtime_error("joint_names cannot be empty.");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
    pcd1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sphere_pointcloud", 10);
    pcd2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sphere_pointcloud", 10);

    mesh_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mesh_vis", 10);
    normals_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("normals_vis", 10);
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);

    current_joint_positions_ = Eigen::VectorXd::Zero(dof_);
    current_cartesian_pose_  = kdl_model_.computeFK(current_joint_positions_, false, tip_frame_);

    RCLCPP_INFO(this->get_logger(), "Initialization completed (DOF=%zu)", dof_);
}

bool RosKinematicsUtility::validateJointVectorSize(const Eigen::VectorXd &q) const
{
    return (size_t)q.size() == dof_;
}

void RosKinematicsUtility::run()
{
    std::string mode;
    while (rclcpp::ok())
    {
        std::cout << "\n=== Robot Control (Rotation Vector Mode) ===\n";
        std::cout << "1 = Joint Space Control\n";
        std::cout << "2 = Cartesian Space Control (Rotation Vector)\n";
        std::cout << "3 = Show Joint Positions\n";
        std::cout << "4 = Show Cartesian Pose (Rotation Vector)\n";
        std::cout << "q = Quit\n";
        std::cout << "Choice: ";

        std::getline(std::cin, mode);

        // ------------ Joint Space ------------
        if (mode == "1")
        {
            std::cout << "Enter " << dof_ << " joint angles (degrees): ";
            std::string in; std::getline(std::cin, in);

            std::istringstream iss(in);
            std::vector<double> vals; double x;
            while (iss >> x) vals.push_back(x);

            if (vals.size() != dof_) {
                std::cout << "Incorrect DOF\n";
                continue;
            }

            Eigen::VectorXd q(dof_);
            for (size_t i = 0; i < dof_; ++i)
                q[i] = vals[i] * M_PI/180.0;

            moveJointSpace(q);
        }

        // ------------ Cartesian Space (Rotation Vector) ------------
        else if (mode == "2")
        {
            std::cout << "Enter x y z rx ry rz (rotation vector, radians): ";
            std::string in; std::getline(std::cin, in);

            std::istringstream iss(in);
            std::vector<double> vals; double x;
            while (iss >> x) vals.push_back(x);

            if (vals.size() != 6) {
                std::cout << "Need 6 values.\n";
                continue;
            }

            Eigen::VectorXd pose(6);
            for (int i = 0; i < 6; i++)
                pose[i] = vals[i];

            moveCartesianSpace(pose);
        }

        else if (mode == "3")
        {
            std::cout << "\nCurrent Joint Positions:\n";
            for (size_t i = 0; i < dof_; i++)
                std::cout << joint_names_[i] << ": "
                          << current_joint_positions_[i] << " rad\n";
        }

        else if (mode == "4")
        {
            std::cout << "\nCurrent Cartesian Pose:\n"
                      << "Position: " << current_cartesian_pose_[0] << ", "
                                     << current_cartesian_pose_[1] << ", "
                                     << current_cartesian_pose_[2] << "\n"
                      << "Rotation Vector: "
                      << current_cartesian_pose_[3] << ", "
                      << current_cartesian_pose_[4] << ", "
                      << current_cartesian_pose_[5] << "\n";
        }

        else if (mode == "q") break;
    }
}

void RosKinematicsUtility::moveJointSpace(const Eigen::VectorXd &q)
{
    if (!validateJointVectorSize(q)) return;

    Eigen::VectorXd fk = kdl_model_.computeFK(q, false, tip_frame_);

    std::cout << "FK (rotation vector): " << fk.transpose() << "\n";

    std::vector<double> joints(dof_);
    for (size_t i = 0; i < dof_; ++i) joints[i] = q[i];

    publishJointState(joints);
}

void RosKinematicsUtility::moveCartesianSpace(const Eigen::VectorXd &pose)
{
    Eigen::VectorXd ik = kdl_model_.computeIK(pose, false, tip_frame_);

    std::cout << "IK (rad): " << ik.transpose() << "\n";

    std::vector<double> joints(dof_, 0.0);
    for (size_t i = 0; i < dof_ && i < (size_t)ik.size(); ++i)
        joints[i] = ik[i];

    publishJointState(joints);
}

void RosKinematicsUtility::publishJointState(const std::vector<double> &pos)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = pos;

    joint_pub_->publish(msg);

    current_joint_positions_ = Eigen::Map<const Eigen::VectorXd>(pos.data(), dof_);
    current_cartesian_pose_  = kdl_model_.computeFK(current_joint_positions_, false, tip_frame_);
}


void RosKinematicsUtility::publishPointCloud(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = this->now();
    cloud.header.frame_id = frame_id;

    cloud.height = 1;
    cloud.width  = points.size();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto& p : points)
    {
        *iter_x = p.x();
        *iter_y = p.y();
        *iter_z = p.z();
        ++iter_x; ++iter_y; ++iter_z;
    }

    pub->publish(cloud);
}

Eigen::VectorXd RosKinematicsUtility::getCurrentJointPositions() const
{
    return current_joint_positions_;
}

Eigen::VectorXd RosKinematicsUtility::getCurrentCartesianPose() const
{
    return current_cartesian_pose_;
}
