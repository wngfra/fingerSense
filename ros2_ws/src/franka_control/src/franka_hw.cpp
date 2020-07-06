#include <exception>
#include <memory>
#include <mutex>
#include <vector>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>

#include "franka_hw.h"

namespace franka_hw
{
    FrankaHW::FrankaHW()
        : cartP_ros2command_({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
          cartV_ros2command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

    bool FrankaHW::init(std::shared_ptr<rclcpp::Node> nh)
    {
        if (initialized_)
        {
            RCLCPP_ERROR(nh->get_logger(), "FrankaHW: Cannot be initialized twice.");
            return false;
        }

        if (!initParameters(nh))
        {
            RCLCPP_ERROR(nh->get_logger(), "FrankaHW: Failed to parse all required parameters.");
            return false;
        }
        try
        {
            initRobot();
        }
        catch (const std::runtime_error &error)
        {
            RCLCPP_ERROR(nh->get_logger(), "FrankaHW: Failed to initialize libfranka robot. %s", error.what());
            return false;
        }

        initialized_ = true;
        return true;
    }

    bool FrankaHW::initParameters(std::shared_ptr<rclcpp::Node> nh)
    {
        bool rate_limiting;
        if (!nh.get_parameters("rate_limiting", rate_limiting))
        {
            RCLCPP_ERROR(nh->get_logger(), "Invalid or no rate_limiting parameter provided");
            return false;
        }

        double cutoff_frequency;
        if (!nh.get_parameters("cutoff_frequency", cutoff_frequency))
        {
            RCLCPP_ERROR(nh->get_logger(), "Invalid or no cutoff_frequency parameter provided");
            return false;
        }

        std::string internal_controller;
        if (!nh.get_parameters("internal_controller", internal_controller))
        {
            RCLCPP_ERROR(nh->get_logger(), "No internal_controller parameter provided");
            return false;
        }

        if (!nh.get_parameters("robot_ip", robot_ip_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Invalid or no robot_ip parameter provided");
            return false;
        }

        // Get full collision behavior config from the parameter server.
        std::vector<double> thresholds =
            getCollisionThresholds("lower_torque_thresholds_acceleration", nh,
                                   {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.lower_torque_thresholds_acceleration.begin());
        thresholds = getCollisionThresholds("upper_torque_thresholds_acceleration", nh,
                                            {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.upper_torque_thresholds_acceleration.begin());
        thresholds = getCollisionThresholds("lower_torque_thresholds_nominal", nh,
                                            {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.lower_torque_thresholds_nominal.begin());
        thresholds = getCollisionThresholds("upper_torque_thresholds_nominal", nh,
                                            {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.upper_torque_thresholds_nominal.begin());
        thresholds.resize(6);
        thresholds = getCollisionThresholds("lower_force_thresholds_acceleration", nh,
                                            {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.lower_force_thresholds_acceleration.begin());
        thresholds = getCollisionThresholds("upper_force_thresholds_acceleration", nh,
                                            {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.upper_force_thresholds_acceleration.begin());
        thresholds = getCollisionThresholds("lower_force_thresholds_nominal", nh,
                                            {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.lower_force_thresholds_nominal.begin());
        thresholds = getCollisionThresholds("upper_force_thresholds_nominal", nh,
                                            {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
        std::copy(thresholds.begin(), thresholds.end(),
                  collision_config_.upper_force_thresholds_nominal.begin());

        return true;
    }

    void initRobot()
    {
        robot_ = std::make_unique<franka::Robot>(robot_ip_);
        model_ = std::make_unique<franka::Model>(robot_->loadModel());

        update(robot_->readOnce());
    }

    FrankaHW::update(const franka::RobotState& robot_state) {
        std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
        robot_state_ros_ = robot_state;
    }

    FrankaHW::control() {}

    FrankaHW::reset() {}

    std::vector<double> FrankaHW::getCollisionThresholds(const std::string &name,
                                                         std::shared_ptr<rclcpp::Node> nh,
                                                         const std::vector<double> &defaults)
    {
        std::vector<double> thresholds;
        if (!nh.get_parameters("collision_config/" + name, thresholds) ||
            thresholds.size() != defaults.size())
        {
            std::string message;
            for (const double &threshold : defaults)
            {
                message += std::to_string(threshold);
                message += " ";
            }
            RCLCPP_INFO(nh->get_logger(), "No parameter %s found, using default values: %s", name.c_str(), message.c_str());
            return defaults;
        }
        return thresholds;
    }

} // namespace franka_hw