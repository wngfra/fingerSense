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
         *
         * TODO sliding motion
         * 
         */

        double x_max = 0.3;
        double time = 0.0;
        double v_max = 0.3;
        double accel_x = v_max * v_max / 1.5; // max_ddx guarantee
        double omega = v_max / accel_x;
        double accel_time = M_PI_2 / omega;
        double const_v_time = (x_max - 2 * accel_x) / v_max;

        int cycle_count = 0;
        int cycle_max = 3;

        // define callback for the cartesian control loop
        std::function<franka::CartesianVelocities(const franka::RobotState &, franka::Duration)> cartesian_velocities_control_callback = [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
            time += period.toSec();

            double v_x = 0.0;

            if (time <= accel_time)
            {
                v_x = v_max * std::sin(omega * time);
            }
            else if (time <= const_v_time + accel_time)
            {
                v_x = v_max;
            }
            else if (time <= const_v_time + 3 * accel_time)
            {
                double t = time - const_v_time;
                v_x = v_max * std::sin(omega * t);
            }
            else if (time <= 2 * const_v_time + 3 * accel_time)
            {
                v_x = -v_max;
            }
            else if (time <= 2 * const_v_time + 4 * accel_time)
            {
                double t = time - 2 * const_v_time;
                v_x = v_max * std::sin(omega * t);
            }

            franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};

            if (time >= 2 * const_v_time + 4 * accel_time)
            {
                time -= 2 * const_v_time + 4 * accel_time;
                cycle_count += 1;
            }

            if (cycle_count >= cycle_max)
            {
                return franka::MotionFinished(output);
            }

            return output;
        };

        // start real-time control loop
        if (accel_x < x_max)
        {
            robot.control(cartesian_velocities_control_callback);
        }

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