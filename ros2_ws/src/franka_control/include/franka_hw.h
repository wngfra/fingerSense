#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace franka_hw
{
    /**
    * This class wraps the functionality of libfranka for controlling Panda robots into the ros_control
    * framework.
    */
    class FrankaHW
    {
    public:
        FrankaHW();
        virtual ~FrankaHW();

        /**
        * Initializes the FrankaHW class to be fully operational. This involves parsing required
        * configurations from the ROS parameter server, connecting to the robot and setting up interfaces
        * for the ros2_control framework.
        *
        * @param[in] nh A node handle in the root namespace of the control node.
        * 
        * @return True if successful, false otherwise.
        */
        virtual bool init(std::shared_ptr<rclcpp::Node> nh);

        /**
        * Reads the parameterization of the hardware class from the ROS parameter server
        * (e.g. arm_id, robot_ip joint_names etc.)
        *
        * @param[in] nh A node handle in the root namespace of the control node..
        *
        * @return True if successful, false otherwise.
        */
        virtual bool initParameters(std::shared_ptr<rclcpp::Node> nh);

        /**
        * Uses the robot_ip_ to connect to the robot via libfranka and loads the libfranka model.
        */
        virtual void initRobot();

        virtual void update();
        virtual void control(std::shared_ptr<rclcpp::Node> nh) const;

    protected:
        /**
        * Parses a set of collision thresholds from the parameter server. The methods returns
        * the default values if no parameter was found or the size of the array did not match
        * the defaults dimension.
        *
        * @param[in] name The name of the parameter to look for.
        * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
        * @param[in] defaults A set of default values that also specify the size the parameter must have
        * to be valid.
        * @return A set parsed parameters if valid parameters where found, the default values otherwise.
        */
        static std::vector<double> getCollisionThresholds(const std::string &name,
                                                          std::shared_ptr<rclcpp::Node> nh,
                                                          const std::vector<double> &defaults);

        struct CollisionConfig
        {
            std::array<double, 7> lower_torque_thresholds_acceleration;
            std::array<double, 7> upper_torque_thresholds_acceleration;
            std::array<double, 7> lower_torque_thresholds_nominal;
            std::array<double, 7> upper_torque_thresholds_nominal;
            std::array<double, 6> lower_force_thresholds_acceleration;
            std::array<double, 6> upper_force_thresholds_acceleration;
            std::array<double, 6> lower_force_thresholds_nominal;
            std::array<double, 6> upper_force_thresholds_nominal;
        };

        CollisionConfig collision_config_;

        std::mutex ros_state_mutex_;
        franka::RobotState robot_state_ros_{};

        franka::CartesianPose cartP_ros2command_;
        franka::CartesianVelocities cartV_ros2command_;

        bool initialized_{false};
        std::atomic_bool controller_active_{false};

        std::unique_ptr<franka::Robot> robot_;
        std::unique_ptr<franka::Model> model_;

        std::string robot_ip_;
    };

} // namespace franka_hw