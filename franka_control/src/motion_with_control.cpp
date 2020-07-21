// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <array>
#include <chrono>
#include <exception>
#include <iostream>
#include <math.h>
#include <memory>
#include <numeric>
#include <stdlib.h>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <rclcpp/rclcpp.hpp>

#include "tactile_msgs/srv/change_state.hpp"
#include "franka_control/common.h"
#include "franka_control/TactileUpdater.h"

using namespace std::chrono_literals;

/**
 * @example motion_with_control.cpp
 * An example showing how to use a joint velocity motion generator and torque control.
 *
 * Additionally, this example shows how to capture and write logs in case an exception is thrown
 * during a motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: %s <robot-hostname>", argv[0]);
        return -1;
    }

    // Prepare a buffer to update tactile signals
    float tactileValue = 0.0;

    // ROS2 initialization
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<TactileUpdater>(&tactileValue);

    // Setup client for ChangeState service
    auto client_node = rclcpp::Node::make_shared("change_state_client");
    auto client = client_node->create_client<tactile_msgs::srv::ChangeState>("/tactile_publisher/change_state");
    auto request = std::make_shared<tactile_msgs::srv::ChangeState::Request>();
    request->transition = 1;

    // Wait 1s for calibration to finish
    std::this_thread::sleep_for(1s);

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(client_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Info %s", result.get()->info);
    }

    // Spin node asynchronously
    std::thread threaded_executor([&]() {
        rclcpp::spin(nh);
    });

    // Compliance parameters
    const double translational_stiffness{150.0};
    const double rotational_stiffness{10.0};
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

    // Controllers
    franka::Robot robot(argv[1], getRealtimeConfig());
    bool has_error = false;
    try
    {
        setDefaultBehavior(robot);
        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        auto desired_pose = robot.readOnce().O_T_EE;
        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
            impedance_control_callback = [&](const franka::RobotState &robot_state,
                                             franka::Duration duration) -> franka::Torques {
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(desired_pose.data()));
            Eigen::Vector3d position_d(initial_transform.translation());
            Eigen::Quaterniond orientation_d(initial_transform.linear());

            // get state variables
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 42> jacobian_array =
                model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.linear());

            // compute error to desired equilibrium pose
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << position - position_d;

            // orientation error
            // "difference" quaternion
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
            {
                orientation.coeffs() << -orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << -transform.linear() * error.tail(3);

            // compute control
            Eigen::VectorXd tau_task(7), tau_d(7);

            // Spring damper system with damping ratio=1
            tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            tau_d << tau_task + coriolis;

            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

            return tau_d_array;
        };

        double time = 0.0;
        robot.control(impedance_control_callback, [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianPose {
            time += period.toSec();

            if (time >= 30.0)
            {
                std::cout << std::endl
                          << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(robot_state.O_T_EE);
            }

            desired_pose = robot_state.O_T_EE_c;
            double delta_z = (tactileValue - 130.0) * 0.00001;
            desired_pose[14] += delta_z;

            return desired_pose;
        });
    }
    catch (const franka::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", e.what());
        has_error = true;
    }

    if (has_error)
    {
        try
        {
            robot.automaticErrorRecovery();
            has_error = false;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully recovered from error.");
        }
        catch (const franka::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s\nAutomatic error recovery failed!", e.what());
        }
    }

    // ROS2 shutdown
    rclcpp::shutdown();
    threaded_executor.join();

    return 0;
}