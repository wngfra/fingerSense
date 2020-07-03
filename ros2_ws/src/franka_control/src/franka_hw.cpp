#inlclude "franka_hw.cpp"

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace franka_hw
{
    FrankaHW::FrankaHW(rclcpp::Node::sharedPtr node_ptr, string &robot_ip)
    {
        if (!has_instance)
        {
            try
            {
                robot = franka::Robot(robot_ip);
                has_error = false;
                has_instance = true;
            }
            catch (franka::Exception const &e)
            {
                has_error = true;
                RCLCPP_WARN(node_ptr->get_logger(), e.what());
            }
        }
    }

    FrankaHW::update()
    {
    }

    FrankaHW::control()
    {
    }

    FrankaHW::reset(rclcpp::Node::sharedPtr node_ptr)
    {
        if (has_error)
        {
            try
            {
                robot.automaticErrorRecovery();
                has_error = false;
            }
            catch (franka::Exception const &e)
            {
                RCLCPP_WARN(node_ptr->get_logger(), e.what());
                RCLCPP_ERROR("Automatic recovery failed.");
            }
        }
    }

} // namespace franka_hw