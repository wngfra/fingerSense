#include <array>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franka_control_interface/control_common.h"
#include "franka_msgs/msg/franka_command.hpp"
#include "franka_msgs/msg/franka_state.hpp"

#ifndef LOGGING
#define LOGGING 1
#endif

using namespace std::chrono_literals;

class FrankaCommandListener : public rclcpp::Node
{
public:
    explicit FrankaCommandListener(const std::string &topic_name, std::array<double, 6> &new_commands) : Node("franka_control_interface")
    {
        auto callback = [&](const franka_msgs::msg::FrankaCommand::SharedPtr msg) -> void {
            if (LOGGING)
                RCLCPP_INFO(this->get_logger(), "Command sent to Franka is (%f, %f, %f, %f, %f, %f)", msg->command[0], msg->command[1], msg->command[2], msg->command[3], msg->command[4], msg->command[5]);
            new_commands = std::move(msg->command);
        };

        sub_ = create_subscription<franka_msgs::msg::FrankaCommand>(topic_name, 10, callback);
    }

private:
    rclcpp::Subscription<franka_msgs::msg::FrankaCommand>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Initialize rclcpp
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    // Connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // Create FrankaCommand subscriber node
    std::array<double, 6> commands{};
    auto topic = std::string("franka_commands");
    auto node_sub = std::make_shared<FrankaCommandListener>(topic, commands);

    // Prepare for robot state publisher
    auto node_pub = rclcpp::Node::make_shared("robot_state_publisher");
    auto robot_state_pub = node_pub->create_publisher<franka_msgs::msg::FrankaState>("robot_states", 10);
    rclcpp::WallRate loop_rate(30);
    franka_msgs::msg::FrankaState robot_state_msg;

    // Robot controller
    std::thread thread([&]() {
        franka::Model model = robot.loadModel();

        while (rclcpp::ok())
        {
            try
            {
                double time = 0.0;
                std::array<double, 6> vt{};
                robot.control([&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                    // Publish robot states
                    robot_state_msg.header.frame_id = "end_effector";
                    robot_state_msg.header.stamp = node_pub->get_clock()->now();
                    robot_state_msg.o_t_ee = robot_state.O_T_EE;
                    robot_state_msg.o_t_ee_c = robot_state.O_ddP_EE_c;
                    robot_state_msg.o_f_ext_hat_k = robot_state.O_F_ext_hat_K;
                    robot_state_msg.tau_j = robot_state.tau_J;
                    robot_state_pub->publish(robot_state_msg);

                    return generateMotion(commands, model, period, robot_state, time, vt);
                });
            }
            catch (const franka::Exception &e)
            {
                RCLCPP_WARN(node_sub->get_logger(), e.what());
                RCLCPP_INFO(node_sub->get_logger(), "Running error recovery...");
                robot.automaticErrorRecovery();
            }
        }
    });

    rclcpp::spin(node_sub);
    rclcpp::shutdown();

    if (thread.joinable())
        thread.join();

    return 0;
}
