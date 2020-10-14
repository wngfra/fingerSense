// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_control/SlidingParameterServer.h"

namespace franka_control
{
    void SlidingParameterServer::change_sliding_parameter(const std::shared_ptr<franka_interfaces::srv::ChangeSlidingParameter::Request> request, std::shared_ptr<franka_interfaces::srv::ChangeSlidingParameter::Response> response)
    {
        *distance_ = request->distance;
        *pressure_ = request->pressure;
        *speed_ = request->speed;

        response->success = true;
    }
} // namespace franka_control
