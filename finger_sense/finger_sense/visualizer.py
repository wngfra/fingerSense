# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
import numpy as np
import rclpy
from matplotlib import pyplot as plt
from matplotlib import animation
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import SlidingControl
from tactile_interfaces.msg import TactileSignal


data_field = np.zeros((4, 4))
map_coords = [(0, 0),
              (1, 0), (0, 1),
              (2, 0), (1, 1), (0, 2),
              (3, 0), (2, 1), (1, 2), (0, 3),
              (3, 1), (2, 2), (1, 3),
              (3, 2), (2, 3),
              (3, 3)]


class Visualizer(Node):

    def __init__(self):
        super().__init__('visualizer')

        self.count = 0
        self.robot_state = [np.zeros(6), np.zeros(16)]

        self.sub_robot = self.create_subscription(
            RobotState,
            'franka_state',
            self.robot_callback,
            100
        )
        self.sub_tactile = self.create_subscription(
            TactileSignal,
            'tactile_signals',
            self.tactile_callback,
            10
        )

        self.fig = plt.figure()

    def robot_callback(self, msg):
        self.robot_state = [msg.o_f_ext_hat_k, msg.o_t_ee]

    def tactile_callback(self, msg):
        raw_data = msg.data
        if raw_data is None:
            self.get_logger().warn("Tactile data is None-Type.")
        else:
            self.count += 1
            for i in range(16):
                y, x = map_coords[i]
                data_field[y, x] = raw_data[i]

            plt.imshow(data_field)
            plt.pause(0.03)


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
