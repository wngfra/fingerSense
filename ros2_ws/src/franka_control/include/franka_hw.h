#pragma once

#include <atomic>

#include <rclcpp/rclcpp.hpp>

namespace franka_hw
{
    // Singleton class for franka hardware abstraction
    class FrankaHW
    {
    public:
        FrankaHW(rclcpp::Node::sharedPtr, string &);
        // ~FrankaHW();

        void update();
        void control();
        void reset(rclcpp::Node::sharedPtr);

    private:
        std::atomic_bool has_error = false;
        std::atomic_bool has_instance = false;
        ;
        franka::Robot robot;
    }

} // namespace franka_hw