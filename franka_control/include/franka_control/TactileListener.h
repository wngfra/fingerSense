#pragma once

#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "tactile_msgs/msg/tactile_signal.hpp"

namespace franka_control
{
    class TactileListener : public rclcpp::Node
    {
    public:
        TactileListener(const std::shared_ptr<std::array<int, 16>> bufPtr) : Node("tactile_listener")
        {   
            bufPtr_ = bufPtr;
            init();
        }

    private:
        void init();
        void topicCallback(const tactile_msgs::msg::TactileSignal::SharedPtr msg);

        rclcpp::Subscription<tactile_msgs::msg::TactileSignal>::SharedPtr subscription_;
    
        std::shared_ptr<std::array<int, 16>> bufPtr_;
    };
} // namespace franka_control
