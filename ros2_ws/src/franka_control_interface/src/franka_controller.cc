#include <array>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franka_control_interface/control_common.h"
#include "franka_msgs/msg/franka_command.hpp"

using namespace std::chrono_literals;

class FrankaCommandListener : public rclcpp::Node
{
public:
    explicit FrankaCommandListener(const std::string &topic_name, std::array<double, 6> &command) : Node("franka_command_listener")
    {
        auto callback = [&command, this](const franka_msgs::msg::FrankaCommand::SharedPtr msg) -> void {
            // RCLCPP_INFO(this->get_logger(), "Command sent to franka is (%f, %f, %f, %f, %f, %f)", msg->command[0], msg->command[1], msg->command[2], msg->command[3], msg->command[4], msg->command[5]);
            command = msg->command;
        };

        sub_ = create_subscription<franka_msgs::msg::FrankaCommand>(topic_name, 10, callback);
    }

private:
    rclcpp::Subscription<franka_msgs::msg::FrankaCommand>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    std::array<double, 6> command{};

    std::thread thread([=, &command, &robot]() {
        franka::Model model = robot.loadModel();
        double time = 0.0;

        robot.control([=, &command, &model, &time](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
            return generateMotion(command, model, period, robot_state, time);
        });
    });

    auto topic = std::string("franka_commands");
    auto node = std::make_shared<FrankaCommandListener>(topic, command);
    rclcpp::spin(node);

    thread.join();
    rclcpp::shutdown();
    return 0;
}