#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

#include <PCANBasic.h>
#include <rclcpp/rclcpp.hpp>

#include "franka_msgs/msg/tactile_signal.hpp"

#define PCAN_DEVICE PCAN_PCIBUS2

using namespace std::chrono_literals;

class Driver : public rclcpp::Node
{
public:
    Driver(const std::string &node_name) : Node(node_name)
    {
        // Lock memory, open CAN port and set filters
        mlockall(MCL_CURRENT | MCL_FUTURE);
        Status = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_1M, 0, 0, 0);
        CAN_FilterMessages(PCAN_DEVICE, 0x405, 0x40b, PCAN_MESSAGE_STANDARD);

        RCLCPP_INFO(
            this->get_logger(), "CAN_Initialize(%xh): Status=0x%x", PCAN_DEVICE, (int)Status);

        // Correct remapping order of the signals' id and taxiles' id
        std::array<uint, 16> channel_order{{11, 15, 14, 12, 9, 13, 8, 10, 6, 7, 4, 5, 2, 0, 3, 1}};

        auto publish = [this, channel_order]() -> void {
            size_t tid, ctid;
            std::array<int32_t, 16> data{};

            // Read sensor signals in bytes and convert to 16 channels of 16bit integers
            for (int i = 0; i < 4; ++i)
            {
                while ((Status = CAN_Read(PCAN_DEVICE, &Message, NULL)) == PCAN_ERROR_QRCVEMPTY)
                {
                    if (usleep(1))
                        break;
                }
                if (Status != PCAN_ERROR_OK)
                {
                    RCLCPP_INFO(
                        this->get_logger(), "CAN_Read(%xh) failure 0x%x", PCAN_DEVICE, (int)Status);
                    break;
                }

                // Compute the starting tactile id
                tid = (int)(Message.ID - 0x405) * 2;
                for (int i = 0; i < 4; ++i)
                {
                    // Correspondent tactile order
                    ctid = channel_order[tid + i];
                    data[ctid] = (data[ctid] << 8) + (int)Message.DATA[2 * i + 1];
                    data[ctid] = (data[ctid] << 8) + (int)Message.DATA[2 * i];
                }
            }

            // Publish to ros2 topic
            msg_ = std::make_unique<franka_msgs::msg::TactileSignal>();
            msg_->header.frame_id = "base";
            msg_->header.stamp = this->get_clock()->now();
            msg_->data = data;
            pub_->publish(std::move(msg_));

            /* Print sensor response on the screen
            RCLCPP_INFO(this->get_logger(), "Tactile readings: %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu %zu",
                        data[0], data[1], data[2], data[3],
                        data[4], data[5], data[6], data[7],
                        data[8], data[9], data[10], data[11],
                        data[12], data[13], data[14], data[15]);
            */
        };

        timer_ = create_wall_timer(30ms, publish);
        pub_ = create_publisher<franka_msgs::msg::TactileSignal>("tactile_signals", 10);
    }

private:
    TPCANMsg Message;
    TPCANStatus Status;
    std::unique_ptr<franka_msgs::msg::TactileSignal> msg_;
    rclcpp::Publisher<franka_msgs::msg::TactileSignal>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Driver>("tactile_sensor_driver_node");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
