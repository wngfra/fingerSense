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

#ifndef LOGGING
#define LOGGING 0
#endif

using namespace std::chrono_literals;

class FrankaCommandListener : public rclcpp::Node
{
public:
    explicit FrankaCommandListener(const std::string &topic_name, FrankaCommand &franka_command) : Node("franka_command_listener")
    {
        auto callback = [&](const franka_msgs::msg::FrankaCommand::SharedPtr msg) -> void {
            if (LOGGING)
                RCLCPP_INFO(this->get_logger(), "Command sent to franka is (%f, %f, %f, %f, %f, %f)", msg->command[0], msg->command[1], msg->command[2], msg->command[3], msg->command[4], msg->command[5]);
            franka_command.set(msg->command);
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
    // Prepare Franka command
    FrankaCommand franka_command;

    // Create FrankaCommand subscriber node
    auto topic = std::string("franka_commands");
    auto node = std::make_shared<FrankaCommandListener>(topic, franka_command);

    // Robot controller
    std::thread thread([&]() {
        franka::Model model = robot.loadModel();
        std::array<double, 6> command{};

        while (!franka_command.is_terminated() && rclcpp::ok())
        {
            // All zero command keeps robot still
            if (!franka_command.is_zero())
            {
                double time = 0.0;
                command = franka_command.fetch();
                try
                {
                    robot.control([&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        return generateMotion(command, model, period, robot_state, time);
                    });
                }
                catch (const franka::Exception &e)
                {
                    RCLCPP_WARN(node->get_logger(), e.what());
                    RCLCPP_INFO(node->get_logger(), "Running error recovery...");
                    robot.automaticErrorRecovery();
                }
            }
            else if (LOGGING)
            {
                RCLCPP_INFO(node->get_logger(), "Remain still.");
            }
        }
    });

    rclcpp::spin(node);
    rclcpp::shutdown();

    if (thread.joinable())
        thread.join();

    return 0;
}
