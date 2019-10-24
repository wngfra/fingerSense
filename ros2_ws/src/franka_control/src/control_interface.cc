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
    explicit FrankaCommandListener(const std::string &topic_name, std::array<double, 6> &new_commands) : Node("franka_command_listener")
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

class FrankaStatePublisher : public rclcpp::Node
{
public:
    explicit FrankaStatePublisher(const std::string &topic_name) : Node("franka_state_publisher")
    {
        auto publish = [&]() {
            
        };

        pub_ = create_publisher<franka_msgs::msg::FrankaState>("franka_states", 10);
    }

private:
    rclcpp::Publisher<franka_msgs::msg::FrankaState>::SharedPtr pub_;
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
    auto node = std::make_shared<FrankaCommandListener>(topic, commands);

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
                    return generateMotion(commands, model, period, robot_state, time, vt);
                });
            }
            catch (const franka::Exception &e)
            {
                RCLCPP_WARN(node->get_logger(), e.what());
                RCLCPP_INFO(node->get_logger(), "Running error recovery...");
                robot.automaticErrorRecovery();
            }
        }
    });

    rclcpp::spin(node);
    rclcpp::shutdown();

    if (thread.joinable())
        thread.join();

    return 0;
}
