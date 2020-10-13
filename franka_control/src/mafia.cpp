// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <chrono>
#include <memory>
#include <stdlib.h>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

#include <rclcpp/rclcpp.hpp>

#include "franka_control/common.h"
#include "franka_control/SlidingControl.h"
#include "franka_control/NodeStateManager.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // ROS2 initialization
    rclcpp::init(argc, argv);
    auto node_state_manager = franka_control::NodeStateManager("tactile_publisher_manager", "/tactile_publisher/change_state");

    // Start recoding data
    auto res = node_state_manager.change_state(1, 3s);

    // Set robot controllers
    bool has_error = false;
    std::string robot_ip = "172.16.0.2";

    franka::Robot robot(robot_ip, getRealtimeConfig());

    try
    {
        setDefaultBehavior(robot);

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);

        double x_max = 0.3;
        double z_max = 0.001;
        double v_x_max = 0.08;
        int cycle_max = 1;

        franka_control::SlidingControl sliding_controller;
        sliding_controller.set_parameter(x_max, z_max, v_x_max, cycle_max);

        // start
        robot.control(sliding_controller);
        //end

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
    res = node_state_manager.change_state(99, 0ns);

    // ROS2 shutdown
    rclcpp::shutdown();

    return 0;
}