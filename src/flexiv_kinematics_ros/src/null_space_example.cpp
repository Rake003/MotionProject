#include "flexiv_kinematics_ros/flexiv_ros_kinematics_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>

/**
 * @brief Compute Moore-Penrose pseudoinverse using SVD
 */
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1e-6)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& sv = svd.singularValues();

    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
    for (int i = 0; i < sv.size(); ++i)
        if (sv(i) > tolerance) S_inv(i, i) = 1.0 / sv(i);

    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}

/**
 * @brief Compute nullspace projector: N = I - J⁺J
 */
Eigen::MatrixXd computeNullSpaceProjector(const Eigen::MatrixXd& jac)
{
    Eigen::MatrixXd Jpinv = pseudoInverse(jac);
    int n = jac.cols();
    return Eigen::MatrixXd::Identity(n, n) - Jpinv * jac;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::string urdf_path = "install/flexiv_kinematics_ros/share/flexiv_kinematics_ros/urdf/flexiv_rizon4s_kinematics.urdf";

    auto node = std::make_shared<FlexivROSKinematicsUtils>(urdf_path);

    // Initial configuration
    Eigen::VectorXd q(7);
    q << 0, 1, 0, 1.57, 0, 0, 0;

    // Start ROS2 spin thread
    std::thread spin_thread([&]() { rclcpp::spin(node); });

    // Move to starting configuration
    node->moveJointSpace(q);

    std::cout << "===== Ready for Nullspace Motion =====\n";
    std::cout << "Press ENTER to move along nullspace.\n";
    std::cout << "Press 'q' + ENTER to quit.\n\n";

    while (true)
    {
        q = node->getCurrentJointPositions();
        std::cout << "\n===== Computing Nullspace Direction =====\n";
        // Compute Jacobian
        auto J = node->kdl_model_flexiv.computeJacobian(q, "Flange");
        // std::cout << "\nJacobian:\n" << J << "\n";

        // SVD for nullspace
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd S = svd.singularValues();
        Eigen::MatrixXd V = svd.matrixV();

        // std::cout << "\nSingular Values:\n" << S.transpose() << "\n";

        // // Null vector (last column)
        Eigen::VectorXd nullvec = V.col(V.cols() - 1);
        // std::cout << "\nNullspace basis vector:\n" << nullvec.transpose() << "\n";

        // // Verify J * n ≈ 0
        // std::cout << "\nCheck J * nullvec ≈ 0:\n" << (J * nullvec).transpose() << "\n\n";

    
        std::string input;
        std::getline(std::cin, input);

        if (input == "q")
            break;

        // Add small step in nullspace
        double step = 0.1;
        q = q + step * nullvec;

        std::cout << "Moving to new configuration:\n" << q.transpose() << "\n";

        node->moveJointSpace(q);
    }

    std::cout << "\nExiting...\n";

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
