// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <exception>

#include "franka_control/SlidingParameterServer.h"

namespace franka_control
{
    void SlidingParameterServer::change_sliding_parameter(const std::shared_ptr<franka_interfaces::srv::ChangeSlidingParameter::Request> request, std::shared_ptr<franka_interfaces::srv::ChangeSlidingParameter::Response> response)
    {
        try
        {
            *distance_ = request->distance;
            *force_ = request->force;
            *speed_ = request->speed;

            response->success = true;

            RCLCPP_INFO(this->get_logger(), "Sliding parameters changed to [distance: %f, force: %f, speed: %f]", *distance_, *force_, *speed_);
        }
        catch (const std::exception &e)
        {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Sliding parameter change service call failed!");
        }
    }
} // namespace franka_control
