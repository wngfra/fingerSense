#include <array>
#include <chrono>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_msgs/msg/franka_state.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    franka::Robot robot(argv[1]);
    franka::RobotState robot_state;

    auto node = rclcpp::Node::make_shared("robot_state_publisher");

    auto robot_state_publisher = node->create_publisher<franka_msgs::msg::FrankaState>("robot_states", 10);

    rclcpp::WallRate loop_rate(100);

    franka_msgs::msg::FrankaState msg;
    msg.header.frame_id = "base";

    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    while (rclcpp::ok())
    {
        robot_state = robot.readOnce();
        msg.header.stamp = clock->now();
        msg.o_t_ee = robot_state.O_T_EE;
        msg.o_t_ee_c = robot_state.O_T_EE_c;
        msg.o_f_ext_hat_k = robot_state.O_F_ext_hat_K;
        msg.tau_j = robot_state.tau_J;
        msg.tau_j_d = robot_state.tau_J_d;

        robot_state_publisher->publish(msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}