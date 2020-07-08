#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "franka_control/TactileListener.h"
#include "tactile_msgs/msg/tactile_signal.hpp"

using std::placeholders::_1;

namespace franka_control
{
    void TactileListener::init()
    {
        subscription_ = this->create_subscription<tactile_msgs::msg::TactileSignal>(
            "/tactile_signals",
            10,
            std::bind(&TactileListener::topicCallback, this, _1));
    }

    void TactileListener::topicCallback(const tactile_msgs::msg::TactileSignal::SharedPtr msg)
    {
        *bufPtr_ = std::move(msg->data);
    }
} // namespace franka_control