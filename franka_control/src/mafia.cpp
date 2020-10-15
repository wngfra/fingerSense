// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <chrono>
#include <memory>
#include <stdlib.h>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <rclcpp/rclcpp.hpp>

#include "franka_control/common.h"
#include "franka_control/SlidingControl.h"
#include "franka_control/SlidingParameterServer.h"
#include "franka_control/NodeStateManager.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    double *distance = new double(0.0);
    double *force = new double(0.0);
    double *speed = new double(0.0);

    // ROS2 initialization
    rclcpp::init(argc, argv);
    auto node_state_manager = franka_control::NodeStateManager("tactile_publisher_manager", "/tactile_publisher/change_state");
    auto server_handler = std::make_shared<franka_control::SlidingParameterServer>(distance, force, speed);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server_handler);
    std::thread thread([&]() {
        executor.spin();
    });

    // Start recoding data
    auto res = node_state_manager.change_state(1, 3s);
    std::this_thread::sleep_for(2s);

    // Set robot controllers
    bool has_error = false;
    std::string robot_ip = "172.16.0.2";

    franka::Robot robot(robot_ip, getRealtimeConfig());

    try
    {
        setDefaultBehavior(robot);
        auto model_ptr = std::make_shared<franka::Model>(robot.loadModel());

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);

        const int cycle_max = 1;

        franka_control::SlidingControl controller(model_ptr);

        while (*speed > 0.0)
        {
            RCLCPP_INFO(server_handler->get_logger(), "distance: %f, force: %f, speed: %f", *distance, *force, *speed);
            controller.set_parameter(*distance, *force, *speed, cycle_max);
            try
            {
                robot.control([&](const franka::RobotState &robot_state, franka::Duration duration) -> franka::Torques {
                    return controller.force_control_callback(robot_state, duration);
                }, controller);
            }
            catch (const franka::Exception &e)
            {
                robot.automaticErrorRecovery();
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Attemped recovery from error: \n%s.", e.what());
                robot.control(motion_generator);
            }
        }

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
    if (res)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shut down tactile signal publisher node.");
    }

    // ROS2 shutdown
    if (thread.joinable())
    {
        thread.join();
    }
    rclcpp::shutdown();

    return 0;
}