// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <algorithm>
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

#include "franka_control/Action.h"
#include "tactile_msgs/srv/change_state.hpp"
#include "franka_control/common.h"
#include "franka_control/TactileUpdater.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // ROS2 initialization
    rclcpp::init(argc, argv);

    // Setup client for ChangeState service
    auto client_node = rclcpp::Node::make_shared("change_state_client");
    auto client = client_node->create_client<tactile_msgs::srv::ChangeState>("/tactile_publisher/change_state");
    auto request = std::make_shared<tactile_msgs::srv::ChangeState::Request>();
    request->transition = 1;

    // Wait 3s for calibration to finish
    std::this_thread::sleep_for(3s);
    // Start recoding data
    if (rclcpp::spin_until_future_complete(client_node, client->async_send_request(request)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call successful!");
    }

    // Set robot controllers
    bool has_error = false;
    std::string robot_ip = "172.16.0.2";

    franka::Robot robot(robot_ip, getRealtimeConfig());
    franka::Model model = robot.loadModel();

    try
    {
        setDefaultBehavior(robot);

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);

        /*
         * Control loop
         * REVIEW: composite action
         * TODO: action gradient
         */
        // Set up action bases
        int n = 31;
        Eigen::MatrixXd bound(6, 2);
        bound << -0.001,          0.001,
                 -0.001,          0.001,
                 -0.001,          0.001,
                 -M_PI_4 / 100, M_PI_4 / 100,
                 -M_PI_4 / 100, M_PI_4 / 100,
                 -M_PI_4 / 100, M_PI_4 / 100;
        Eigen::MatrixXd weights(6, n);
        std::vector<std::function<double(const double &)>> bases;
        franka_control::Action action(bound);

        for (int i = 0; i < n; ++i)
        {
            double omega = (i + 1) / 2;
            double coeff = (i + 1) % 2;

            weights.col(i) << Eigen::VectorXd::Random(6) * 1e-5;
            bases.push_back([&](const double &t) -> double {
                return (1.0 - coeff) * sin(omega * 10 * M_PI * t) + coeff * cos(omega * 10 * M_PI * t);
            });
        }
        

        bool res = action.addBases(bases, weights);
        auto generated_pos = action(0.0);

        // Compliance parameters
        double translational_stiffness{150.0};
        double rotational_stiffness{60.0};
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                           Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                               Eigen::MatrixXd::Identity(3, 3);

        franka::RobotState initial_state = robot.readOnce();
        auto desired_pose = initial_state.O_T_EE;

        double time = 0.0;
        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)> cartesian_impedance_control_callback = [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
            time += period.toSec();

            // equilibrium point is moving down
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

            if (time >= 30.0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finished motion, shutting down controller");
                franka::Torques torques(tau_d_array);
                return franka::MotionFinished(torques);
            }

            generated_pos = action(time);
            desired_pose[12] += generated_pos[0];
            desired_pose[13] += generated_pos[1];
            desired_pose[14] += generated_pos[2];
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dx: %f", generated_pos[0]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dy: %f", generated_pos[1]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dz: %f", generated_pos[2]);

            return tau_d_array;
        };

        // start real-time control loop
        robot.control(cartesian_impedance_control_callback);

        RCLCPP_INFO(rclcpp::get_logger("libfranka"), "Touched the platform!");

        robot.control(motion_generator);
    }
    catch (const franka::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("libfranka"), "%s", e.what());
        has_error = true;
    }

    // Recover from error
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

    // Shutdown tactile signal publisher node
    request->transition = 99;
    if (rclcpp::spin_until_future_complete(client_node, client->async_send_request(request)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call successful!");
    }

    // ROS2 shutdown
    rclcpp::shutdown();

    return 0;
}