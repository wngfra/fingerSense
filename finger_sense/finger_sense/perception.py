# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import matplotlib.pyplot  as plt
import numpy as np
import rclpy
from rclpy.node import Node

from tactile_msgs.msg import TactileSignal


class Perception(Node):

    def __init__(self):
        super().__init__("tactile_perception")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('stack_size', 64)
            ]
        )

        self.__count = 0
        self.__size = self.get_parameter(
            'stack_size').get_parameter_value().integer_value
        self.__x = np.zeros((self.__size, 16), dtype=np.float32)

        self.__subscription = self.create_subscription(
            TactileSignal,
            '/tactile_signals',
            self.percept_callback,
            10
        )

        self.fig = plt.figure()

    def percept_callback(self, msg):
        x = np.copy(self.__x)
        self.append_data(msg.data)

        dx = self.__x - x
        dx = (dx + 10.0) / 500.0 * 255.0
        x = (x + 10.0) / 500.0 * 255.0

        if self.__count >= self.__size:
            plt.subplot(1,2,1)
            plt.imshow(x)
            plt.subplot(1,2,2)
            plt.imshow(dx)
            plt.pause(0.03)
            plt.clf()

    def append_data(self, item):
        if self.__count < self.__size:
            self.__x[self.__count, :] = item
        else:
            self.__x[:-1, :] = self.__x[1:, :]
            self.__x[-1] = item

        self.__count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()