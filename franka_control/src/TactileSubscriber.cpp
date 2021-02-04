// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <numeric>

#include "franka_control/TactileSubscriber.h"

namespace franka_control
{
    void TactileSubscriber::topic_callback(const tactile_interfaces::msg::TactileSignal::SharedPtr msg)
    {
        auto data_array = msg->data;
        *fp_ = (float)(data_array.front() - data_array.back());
    }

} // namespace franka_control