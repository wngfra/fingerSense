// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "franka_interfaces/srv/change_sliding_parameter.hpp"

namespace franka_control
{
    class SlidingParameterServer : public rclcpp::Node
    {
    public:
        SlidingParameterServer(double *distance, double *dz, double *speed) : Node("sliding_parameter_server")
        {
            distance_ = distance;
            dz_ = dz;
            speed_ = speed;

            service_ = this->create_service<franka_interfaces::srv::ChangeSlidingParameter>("change_sliding_parameter", std::bind(&SlidingParameterServer::change_sliding_parameter, this, std::placeholders::_1, std::placeholders::_2));
        }

    private:
        void change_sliding_parameter(const std::shared_ptr<franka_interfaces::srv::ChangeSlidingParameter::Request>, std::shared_ptr<franka_interfaces::srv::ChangeSlidingParameter::Response>);

        double *distance_, *dz_, *speed_;
        rclcpp::Service<franka_interfaces::srv::ChangeSlidingParameter>::SharedPtr service_;
    };
} // namespace franka_control
