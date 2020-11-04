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
#include "franka_control/FrankaStatePublisher.h"
#include "franka_control/SlidingControl.h"
#include "franka_control/SlidingParameterServer.h"
#include "franka_control/NodeStateManager.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    double *distance = new double(0.0);
    double *force = new double(0.0);
    double *speed = new double(0.0);

    auto O_F_ext_hat_K = std::make_shared<std::array<double, 6>>();
    auto position = std::make_shared<std::array<double, 3>>();
    auto quaternion = std::make_shared<std::array<double, 4>>();

    // ROS2 initialization
    rclcpp::init(argc, argv);
    auto publisher_handler = std::make_shared<franka_control::FrankaStatePublisher>(O_F_ext_hat_K, position, quaternion);
    auto node_state_manager = franka_control::NodeStateManager("tactile_publisher_manager", "/tactile_publisher/change_state");
    auto server_handler = std::make_shared<franka_control::SlidingParameterServer>(distance, force, speed);
    std::thread thread([&]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(publisher_handler);
        executor.add_node(server_handler);
        executor.spin();
    });

    // Set robot controllers
    bool has_error = false;
    std::string robot_ip = "172.16.0.2";

    franka::Robot robot(robot_ip, getRealtimeConfig());
    std::array<double, 7> q_goal = {{-0.000197684, 0.35463, 0.000567185, -2.73805, -0.000571208, M_PI, 0.785693}};
    MotionGenerator motion_generator(0.5, q_goal);

    try
    {
        setDefaultBehavior(robot);
        auto model_ptr = std::make_shared<franka::Model>(robot.loadModel());
        franka_control::SlidingControl sliding_controller(model_ptr);

        // First move the robot to a suitable joint configuration
        robot.control(motion_generator);

        // Start recoding data
        node_state_manager.change_state(50, 3s);

        sliding_controller.set_stiffness({{200, 200, 200, 20, 20, 20}}, 1.0);
        robot.control(
            [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
                getFrankaState(robot_state, *O_F_ext_hat_K, *position, *quaternion);
                return sliding_controller.touch_control_callback(robot_state, period);
            });

        RCLCPP_INFO(rclcpp::get_logger("mafia"), "Touched the platform.");

        node_state_manager.change_state(1, 0s);
        std::this_thread::sleep_for(3s);

        sliding_controller.set_stiffness({{3500, 300, 1000, 300, 300, 300}}, 1.0);

        while (*speed > 0.0)
        {
            RCLCPP_INFO(server_handler->get_logger(), "distance: %f, force: %f, speed: %f", *distance, *force, *speed);
            sliding_controller.set_sliding_parameter(*distance, *force, *speed, 1);
            robot.control(
                [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
                    getFrankaState(robot_state, *O_F_ext_hat_K, *position, *quaternion);
                    return sliding_controller.force_control_callback(robot_state, period);
                },
                [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                    return sliding_controller(robot_state, period);
                });
        }
    }
    catch (const franka::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mafia"), "%s", e.what());
        has_error = true;
    }

    // Recover from error
    if (has_error)
    {
        try
        {
            robot.automaticErrorRecovery();
            has_error = false;
            RCLCPP_INFO(rclcpp::get_logger("mafia"), "Successfully recovered from error.");
        }
        catch (const franka::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("mafia"), "%s\nAutomatic error recovery failed!", e.what());
        }
    }
    robot.control(motion_generator);

    // Shutdown tactile signal publisher node
    node_state_manager.change_state(99, 0s);

    // ROS2 shutdown
    if (thread.joinable())
    {
        thread.join();
    }
    rclcpp::shutdown();

    return 0;
}