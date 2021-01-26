// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_control/TactileSubscriber.h"

namespace franka_control
{
    void TactileSubscriber::topic_callback(const tactile_interfaces::msg::TactileSignal::SharedPtr msg)
    {
        *data_holder_ = msg->data;
    }

} // namespace franka_control