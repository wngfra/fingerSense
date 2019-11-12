#include <array>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
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
    explicit FrankaCommandListener(const std::string &topic_name, std::array<double, 6> &commands, double &response_time) : Node("franka_control_interface")
    {
        auto callback = [&](const franka_msgs::msg::FrankaCommand::SharedPtr msg) -> void {
            commands = std::move(msg->command);
            response_time = std::move(msg->response_time);
            if (LOGGING)
            {
                RCLCPP_INFO(this->get_logger(), "Command sent to Franka is [%f, %f, %f, %f, %f, %f]",
                            commands[0],
                            commands[1],
                            commands[2],
                            commands[3],
                            commands[4],
                            commands[5]);
                RCLCPP_INFO(this->get_logger(), "Response time set to [%f]s.",
                            response_time);
            }
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
    double response_time = 0.3;
    std::array<double, 6> commands{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    auto topic = std::string("franka_commands");
    auto node_sub = std::make_shared<FrankaCommandListener>(topic, commands, response_time);

    // Prepare for robot state publisher
    auto node_pub = rclcpp::Node::make_shared("robot_state_publisher");
    auto robot_state_pub = node_pub->create_publisher<franka_msgs::msg::FrankaState>("robot_states", 10);
    rclcpp::WallRate loop_rate(30);
    franka_msgs::msg::FrankaState robot_state_msg;

    // Robot controller thread
    std::thread thread([&]() {
        double time = 0.3;
        std::array<double, 6> v_init{};
        franka::Model model = robot.loadModel();

        while (rclcpp::ok())
        {
            try
            {
                // Continuous controller
                robot.control([&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                    if (time >= response_time)
                    {
                        time = 0.0;
                    }

                    // Compute the current Cartesian velocity
                    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
                    Eigen::Matrix<double, 6, 1> v0 = jacobian * dq;

                    // Convert from Eigen
                    std::array<double, 6> v0_array{};
                    Eigen::VectorXd::Map(&v0_array[0], 6) = v0;

                    // Assign initial velocity for Cartesian velocity command computation
                    if (time == 0.0)
                        v_init = v0_array;

                    // Publish robot states
                    robot_state_msg.header.frame_id = "end_effector";
                    robot_state_msg.header.stamp = node_pub->get_clock()->now();
                    robot_state_msg.o_t_ee = robot_state.O_T_EE;
                    robot_state_msg.o_t_ee_c = robot_state.O_T_EE_c;
                    robot_state_msg.o_f_ext_hat_k = robot_state.O_F_ext_hat_K;
                    robot_state_msg.tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
                    robot_state_msg.v_ee = v0_array;
                    robot_state_pub->publish(robot_state_msg);

                    // Compute the Cartesian velocity command
                    return generateMotion(commands, period, time, response_time, v_init);
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
