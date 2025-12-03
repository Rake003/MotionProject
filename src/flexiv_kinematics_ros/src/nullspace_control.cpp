#include "flexiv_kinematics_ros/flexiv_ros_kinematics_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <termios.h>
#include <unistd.h>

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
    int n = jacobian.cols();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    return I - J_pinv * jacobian;
}

/**
 * @brief Get keyboard input without waiting for Enter
 */
char getch()
{
    char buf = 0;
    struct termios old;
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::string urdf_path = "/home/controls/Rakesh/Devel/Redundancy_/cpp/redundancies_ws/install/flexiv_kinematics_ros/share/flexiv_kinematics_ros/urdf/flexiv_rizon4s_kinematics.urdf";

    auto node = std::make_shared<FlexivROSKinematicsUtils>(urdf_path);
    
    std::thread spin_thread([&]() { rclcpp::spin(node); });

    // Initial configuration
    Eigen::VectorXd q_current(7);
    q_current << 0, 1, 0, 1.57, 0, 0, 0;
    
    // Move to initial position and publish
    std::vector<double> initial_joints(7);
    for (int i = 0; i < 7; i++) initial_joints[i] = q_current[i];
    node->publishJointState(initial_joints);
    
    std::cout << "\n=== Interactive Null Space Motion Demo ===" << std::endl;
    std::cout << "Initial joint configuration set." << std::endl;
    std::cout << "Waiting 2 seconds for RViz to update..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Compute initial end-effector pose (this is what we'll maintain)
    Eigen::VectorXd target_ee_pose = node->kdl_model_flexiv.computeFK(q_current, true, "Flange");
    std::cout << "\nTarget EE pose (will be maintained): " << target_ee_pose.transpose() << std::endl;
    
    // Control parameters
    double null_space_gain = 0.0;  // User-controlled gain
    double dt = 0.05;  // Time step (20 Hz)
    int selected_joint = 0;  // Which joint to control in null space
    
    std::cout << "\n=== Controls ===" << std::endl;
    std::cout << "Arrow Keys:" << std::endl;
    std::cout << "  ↑ : Increase null space gain (move towards zero)" << std::endl;
    std::cout << "  ↓ : Decrease null space gain (move away from zero)" << std::endl;
    std::cout << "  → : Select next joint" << std::endl;
    std::cout << "  ← : Select previous joint" << std::endl;
    std::cout << "Space : Reset null space gain to zero" << std::endl;
    std::cout << "r     : Reset to initial configuration" << std::endl;
    std::cout << "m     : Toggle mode (single joint / all joints to mid-range)" << std::endl;
    std::cout << "q     : Quit" << std::endl;
    std::cout << "\n=== Starting Control Loop ===" << std::endl;
    
    bool running = true;
    bool mode_single_joint = true;  // true = single joint, false = all to mid-range
    
    // Non-blocking input setup
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    auto last_time = std::chrono::steady_clock::now();
    
    while (running && rclcpp::ok())
    {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_time;
        
        if (elapsed.count() >= dt)
        {
            last_time = current_time;
            
            // Compute Jacobian at current configuration
            Eigen::MatrixXd J = node->kdl_model_flexiv.computeJacobian(q_current, "Flange");
            Eigen::MatrixXd N = computeNullSpaceProjector(J);
            
            // Design null space velocity based on mode
            Eigen::VectorXd q_null_dot(7);
            
            if (mode_single_joint)
            {
                // Move selected joint towards zero
                q_null_dot = Eigen::VectorXd::Zero(7);
                q_null_dot(selected_joint) = -null_space_gain * q_current(selected_joint);
            }
            else
            {
                // Move all joints towards mid-range (zero)
                Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(7);
                q_null_dot = null_space_gain * (q_desired - q_current);
            }
            
            // Project into null space
            Eigen::VectorXd q_dot = N * q_null_dot;
            
            // Integrate
            q_current += q_dot * dt;
            
            // Publish to RViz
            std::vector<double> joints(7);
            for (int i = 0; i < 7; i++) joints[i] = q_current[i];
            node->publishJointState(joints);
            
            // Compute current EE pose and error
            Eigen::VectorXd current_ee_pose = node->kdl_model_flexiv.computeFK(q_current, true, "Flange");
            Eigen::Vector3d pos_error = current_ee_pose.head<3>() - target_ee_pose.head<3>();
            double pos_error_mm = pos_error.norm() * 1000.0;
            
            // Display status
            std::cout << "\r" << std::flush;
            std::cout << "Mode: " << (mode_single_joint ? "Single Joint" : "All to Mid-range") << " | ";
            std::cout << "Joint " << (selected_joint + 1) << " | ";
            std::cout << "Gain: " << std::fixed << std::setprecision(2) << null_space_gain << " | ";
            std::cout << "Joint angles (deg): ";
            for (int i = 0; i < 7; i++)
            {
                std::cout << std::setw(6) << std::setprecision(1) << q_current[i] * 180.0 / M_PI << " ";
            }
            std::cout << "| EE error: " << std::setw(6) << std::setprecision(2) << pos_error_mm << " mm   ";
            std::cout << std::flush;
        }
        
        // Check for keyboard input (non-blocking)
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0)
        {
            char c = getch();
            
            if (c == 'q' || c == 'Q')
            {
                running = false;
                std::cout << "\nExiting..." << std::endl;
            }
            else if (c == 27)  // Escape sequence for arrow keys
            {
                getch();  // Skip '['
                char arrow = getch();
                
                if (arrow == 'A')  // Up arrow
                {
                    null_space_gain += 0.1;
                    if (null_space_gain > 2.0) null_space_gain = 2.0;
                }
                else if (arrow == 'B')  // Down arrow
                {
                    null_space_gain -= 0.1;
                    if (null_space_gain < -2.0) null_space_gain = -2.0;
                }
                else if (arrow == 'C')  // Right arrow
                {
                    selected_joint = (selected_joint + 1) % 7;
                }
                else if (arrow == 'D')  // Left arrow
                {
                    selected_joint = (selected_joint - 1 + 7) % 7;
                }
            }
            else if (c == ' ')
            {
                null_space_gain = 0.0;
                std::cout << "\nNull space gain reset to zero" << std::endl;
            }
            else if (c == 'r' || c == 'R')
            {
                q_current << 0, 1, 0, 1.57, 0, 0, 0;
                null_space_gain = 0.0;
                std::cout << "\nReset to initial configuration" << std::endl;
            }
            else if (c == 'm' || c == 'M')
            {
                mode_single_joint = !mode_single_joint;
                std::cout << "\nMode switched to: " << (mode_single_joint ? "Single Joint" : "All to Mid-range") << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    std::cout << "\n\n=== Final Summary ===" << std::endl;
    std::cout << "Final joint configuration (deg): ";
    for (int i = 0; i < 7; i++)
    {
        std::cout << q_current[i] * 180.0 / M_PI << " ";
    }
    std::cout << std::endl;
    
    Eigen::VectorXd final_ee_pose = node->kdl_model_flexiv.computeFK(q_current, true, "Flange");
    Eigen::Vector3d final_pos_error = final_ee_pose.head<3>() - target_ee_pose.head<3>();
    
    std::cout << "Final EE position error: " << final_pos_error.norm() * 1000.0 << " mm" << std::endl;

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}