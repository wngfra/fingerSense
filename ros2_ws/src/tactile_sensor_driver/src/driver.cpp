#include <chrono>
#include <memory>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "PCANBasic.h"

#include "rclcpp/rclcpp.hpp"

#include "tactile_sensor_msgs/msg/tactile_signal.hpp"

#define PCAN_DEVICE PCAN_USBBUS1

using namespace std::chrono_literals;


class Driver : public rclcpp::Node
{
public:
    Driver() : Node("tactile_sensor_driver")
    {
        mlockall(MCL_CURRENT | MCL_FUTURE);
        Status = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_1M, 0, 0, 0);

        RCLCPP_INFO(
            this->get_logger(), "CAN_Initialize(%xh): Status=0x%x", PCAN_DEVICE, (int)Status);

        // Read sensor signals and publish
        std::array<uint, 16> channel_order = {{11, 15, 14, 12, 9, 13, 8, 10, 6, 7, 4, 5, 2, 0, 3, 1}};

        auto publish = [this, channel_order]() -> void {
            size_t count = 0;
            size_t order = 0;
            size_t sid;
            std::vector<int> proximity(2, 0);
            std::vector<int> pressure(16, 0);

            while (count < 5)
            {
                while ((Status = CAN_Read(PCAN_DEVICE, &Message, NULL)) == PCAN_ERROR_QRCVEMPTY)
                {
                    if (usleep(100))
                        break;
                }
                if (Status != PCAN_ERROR_OK)
                {
                    RCLCPP_INFO(
                        this->get_logger(), "CAN_Read(%xh) failure 0x%x", PCAN_DEVICE, (int)Status);
                }

                sid = (int)Message.ID;

                if (sid == 0x405 && count == 0)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i + 1];
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i];
                        order += 1;
                    }
                    count = 1;
                }
                else if (sid == 0x407 && count == 1)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i + 1];
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i];
                        order += 1;
                    }
                    count = 2;
                }
                else if (sid == 0x409 && count == 2)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i + 1];
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i];
                        order += 1;
                    }
                    count = 3;
                }
                else if (sid == 0x40b && count == 3)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i + 1];
                        pressure[channel_order[order]] = (pressure[channel_order[order]] << 8) + (int)Message.DATA[2 * i];
                        order += 1;
                    }
                    count = 4;
                }
                else if (sid == 0x601 && count == 4)
                {
                    for (int i = 0; i < 2; ++i)
                    {
                        proximity[i] = (proximity[i] << 8) + (int)Message.DATA[2 * i + 1];
                        proximity[i] = (proximity[i] << 8) + (int)Message.DATA[2 * i];
                    }
                    count = 5;
                }
            }
            msg_ = std::make_unique<tactile_sensor_msgs::msg::TactileSignal>();
            msg_->stamp = this->get_clock()->now(); 
            msg_->pressure = pressure;
            msg_->proximity = proximity[1] - proximity[0];
            pub_->publish(std::move(msg_));

            RCLCPP_INFO(this->get_logger(), "proximity: %d, pressure: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                proximity[1] - proximity[0],
                pressure[0],  pressure[1],  pressure[2],  pressure[3],
                pressure[4],  pressure[5],  pressure[6],  pressure[7],
                pressure[8],  pressure[9],  pressure[10], pressure[11],
                pressure[12], pressure[13], pressure[14], pressure[15]);
        };

        timer_ = create_wall_timer(1ms, publish);
        pub_ = create_publisher<tactile_sensor_msgs::msg::TactileSignal>("tactile_signal", 10);
    }

private:
    TPCANMsg Message;
    TPCANStatus Status;
    std::unique_ptr<tactile_sensor_msgs::msg::TactileSignal> msg_;
    rclcpp::Publisher<tactile_sensor_msgs::msg::TactileSignal>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Driver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
