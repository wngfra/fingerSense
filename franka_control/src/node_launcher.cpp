// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "franka_control/MotionControlServer.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto nh = std::make_shared<franka_control::MotionControlServer>(argv[1]);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}