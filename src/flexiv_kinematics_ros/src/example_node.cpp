#include "flexiv_kinematics_ros/flexiv_ros_kinematics_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>

/**
 * @brief Compute the Moore-Penrose pseudoinverse of a matrix
 */
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1e-6)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    const auto& singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
    
    for (int i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = 1.0 / singularValues(i);
        }
    }
    
    return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
}

/**
 * @brief Compute null space projector: N = (I - J_pinv * J)
 */
Eigen::MatrixXd computeNullSpaceProjector(const Eigen::MatrixXd& jacobian)
{
    Eigen::MatrixXd J_pinv = pseudoInverse(jacobian);
    int n = jacobian.cols(); // number of joints
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    return I - J_pinv * jacobian;
}

/**
 * @brief Design null space velocity for secondary task
 */
Eigen::VectorXd designNullSpaceVelocity(const Eigen::VectorXd& q_current, 
                                         const std::string& task = "mid_range")
{
    Eigen::VectorXd q_null_dot(7);
    
    if (task == "mid_range")
    {
        Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(7);
        double k_null = 0.5;
        q_null_dot = k_null * (q_desired - q_current);
    }
    else if (task == "minimize_joint1")
    {
        q_null_dot = Eigen::VectorXd::Zero(7);
        q_null_dot(0) = -0.1 * q_current(0);
    }
    else if (task == "custom")
    {
        Eigen::VectorXd q_desired(7);
        q_desired << 0.5, -0.5, 0.3, -1.0, 0.2, 0.8, 0.0;
        double k_null = 0.3;
        q_null_dot = k_null * (q_desired - q_current);
    }
    else
    {
        q_null_dot = Eigen::VectorXd::Zero(7);
    }
    
    return q_null_dot;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::string urdf_path =
        "/home/controls/Rakesh/Devel/Redundancy_/cpp/redundancies_ws/install/"
        "flexiv_kinematics_ros/share/flexiv_kinematics_ros/urdf/flexiv_rizon4s_kinematics.urdf";

    auto node = std::make_shared<FlexivROSKinematicsUtils>(urdf_path);

    // Test poses
    Eigen::VectorXd J7rad(7);
    J7rad << 0, 1, 0, 1.57, 0, 0, 0;

    std::thread spin_thread([&]() { rclcpp::spin(node); });

    node->moveJointSpace(J7rad);

    // #### Performing redundancy resolution ####
    std::cout << "=== Redundancy Resolution Demo ===" << std::endl;
    
    // Step 1: Get current joint configuration
    Eigen::VectorXd q_current = node->getCurrentJointPositions();
    std::cout << "Current joint positions (rad): " << q_current.transpose() << std::endl;
    
    // Step 2: Compute current end-effector pose
    Eigen::VectorXd current_ee_pose = node->getCurrentCartesianPose();
    std::cout << "Current EE pose: " << current_ee_pose.transpose() << std::endl;
    
    // Step 3: Compute Jacobian at current configuration
    Eigen::MatrixXd J = node->kdl_model_flexiv.computeJacobian(q_current, "Flange");
    std::cout << "\nJacobian (6x7):\n" << J << std::endl;
    
    // Step 4: Define Cartesian velocity (zero to maintain pose)
    Eigen::VectorXd x_dot = Eigen::VectorXd::Zero(6);
    std::cout << "\nDesired Cartesian velocity (zero): " << x_dot.transpose() << std::endl;
    
    // Step 5: Compute pseudoinverse of Jacobian
    Eigen::MatrixXd J_pinv = pseudoInverse(J);
    std::cout << "\nJacobian pseudoinverse (7x6):\n" << J_pinv << std::endl;
    
    // Step 6: Compute null space projector
    Eigen::MatrixXd N = computeNullSpaceProjector(J);
    std::cout << "\nNull space projector (7x7):\n" << N << std::endl;
    
    // Verify null space: J * N should be approximately zero
    Eigen::MatrixXd J_N = J * N;
    std::cout << "\nVerification (J * N, should be ~0):\n" << J_N << std::endl;
    
    // Step 7: Design null space velocity
    Eigen::VectorXd q_null_dot = designNullSpaceVelocity(q_current, "mid_range");
    std::cout << "\nDesigned null space velocity: " << q_null_dot.transpose() << std::endl;
    
    // Step 8: Compute total joint velocity
    Eigen::VectorXd q_dot = J_pinv * x_dot + N * q_null_dot;
    std::cout << "\nTotal joint velocity (rad/s): " << q_dot.transpose() << std::endl;
    
    // Step 9: Simulate motion with null space projection
    std::cout << "\n=== Simulating Null Space Motion ===" << std::endl;
    double dt = 0.1;
    int num_steps = 50;
    
    Eigen::VectorXd q_simulated = q_current;
    
    std::cout << "Step | Joint Config (deg) | EE Position (m) | EE Position Error (mm)" << std::endl;
    std::cout << std::string(100, '-') << std::endl;
    
    for (int step = 0; step < num_steps; ++step)
    {
        // Recompute Jacobian and null space at current configuration
        J = node->kdl_model_flexiv.computeJacobian(q_simulated, "Flange");
        N = computeNullSpaceProjector(J);
        
        // Recompute null space velocity
        q_null_dot = designNullSpaceVelocity(q_simulated, "mid_range");
        
        // Compute joint velocity (pure null space motion)
        q_dot = N * q_null_dot;
        
        // Integrate to get new joint positions
        q_simulated += q_dot * dt;
        
        // Compute new end-effector pose
        Eigen::VectorXd ee_pose_sim = node->kdl_model_flexiv.computeFK(q_simulated, true, "Flange");
        
        // Compute position error from original pose
        Eigen::Vector3d pos_error = ee_pose_sim.head<3>() - current_ee_pose.head<3>();
        double pos_error_norm = pos_error.norm() * 1000; // in mm
        
        // Print every 10 steps
        if (step % 10 == 0)
        {
            std::cout << std::setw(4) << step << " | ";
            for (int i = 0; i < 7; ++i)
            {
                std::cout << std::setw(6) << std::fixed << std::setprecision(1) 
                          << q_simulated(i) * 180.0 / M_PI << " ";
            }
            std::cout << "| ";
            std::cout << std::setw(6) << std::fixed << std::setprecision(3) 
                      << ee_pose_sim(0) << " " << ee_pose_sim(1) << " " << ee_pose_sim(2);
            std::cout << " | " << std::setw(8) << std::fixed << std::setprecision(3) 
                      << pos_error_norm << std::endl;
        }
    }
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Initial joint config (deg): ";
    for (int i = 0; i < 7; ++i)
    {
        std::cout << q_current(i) * 180.0 / M_PI << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Final joint config (deg):   ";
    for (int i = 0; i < 7; ++i)
    {
        std::cout << q_simulated(i) * 180.0 / M_PI << " ";
    }
    std::cout << std::endl;
    
    std::cout << "\nJoint changes (deg): ";
    for (int i = 0; i < 7; ++i)
    {
        std::cout << (q_simulated(i) - q_current(i)) * 180.0 / M_PI << " ";
    }
    std::cout << std::endl;
    
    Eigen::VectorXd final_ee_pose = node->kdl_model_flexiv.computeFK(q_simulated, true, "Flange");
    Eigen::Vector3d final_pos_error = final_ee_pose.head<3>() - current_ee_pose.head<3>();
    
    std::cout << "\nEnd-effector position error (mm): " 
              << final_pos_error.norm() * 1000.0 << std::endl;
    std::cout << "This demonstrates null space motion: joints moved but EE stayed nearly fixed!" 
              << std::endl;

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}