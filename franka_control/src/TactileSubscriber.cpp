// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_control/TactileSubscriber.h"

#include <numeric>

namespace franka_control
{
    void TactileSubscriber::topic_callback(const tactile_interfaces::msg::TactileSignal::SharedPtr msg)
    {
        auto data_array = msg->data;
        *fp_ = std::accumulate(data_array.begin(), data_array.end(), 0) / 16.0;
        
        RCLCPP_INFO(get_logger(), "average force: %f", *fp_);
    }

} // namespace franka_control