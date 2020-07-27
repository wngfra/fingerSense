# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np
import rclpy
from rclpy.node import Node

from tactile_msgs.msg import TactileSignal



class TactilePercept(Node):

    def __init__(self):
        super().__init__("tactile_percept")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('stack_size', 33)
            ]
        )

        self.__stack_size = self.get_parameter(
            'stack_size').get_parameter_value().integer_value
        self.__stack_count = 0

        self.__stack = np.zeros((self.__stack_size, 16))
        self.__freq = self.__stack

        self.__subscription = self.create_subscription(
            TactileSignal, '/tactile_signals', self.percept_callback, 10)

        # TODO realtime analysis
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines = self.ax.plot(self.__stack)


    def percept_callback(self, msg):
        new_data = msg.data
        self.append_stack(new_data)

        # TODO realtime analysis
        self.__freq = np.fft.fft(self.__stack, axis=0, norm='ortho')

        for i, line in enumerate(self.lines):
            line.set_xdata(np.arange(self.__stack_size))
            line.set_ydata(self.__freq[:, i])
        plt.tight_layout()
        self.fig.canvas.draw()


    def append_stack(self, item):
        if self.__stack_count < self.__stack_size:
            self.__stack[self.__stack_count, :] = item
            self.__stack_count += 1
        else:
            self.__stack[:-1, :] = self.__stack[1:, :]
            self.__stack[-1] = item


def main(args=None):
    rclpy.init(args=args)
    percept_node = TactilePercept()
    rclpy.spin(percept_node)
    percept_node.destroy_node()
    rclpy.shutdiwn()


if __name__ == "__main__":
    main()
