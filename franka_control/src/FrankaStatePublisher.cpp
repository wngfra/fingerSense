// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_control/FrankaStatePublisher.h"

namespace franka_control
{
    void FrankaStatePublisher::timer_callback()
    {
        auto msg = franka_interfaces::msg::RobotState();
        msg.header.frame_id = "base";
        msg.header.stamp = this->get_clock()->now();
        msg.motion_finished = *bMotionFinished_;
        msg.o_f_ext_hat_k = *O_F_ext_hat_K_;
        msg.position = *position_;
        msg.quaternion = *quaternion_;

        publisher_->publish(msg);
    }
} // namespace franka_control
