# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
import rclpy
from mayavi import mlab
from rclpy.node import Node

from franka_interfaces.srv import ChangeSlidingParameter
from tactile_interfaces.msg import TactileSignal

from percepy import fourier_cov

class PerceptionAgent(Node):

    def __init__(self):
        super().__init__('perception_agent')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('stack_size', 64)
            ]
        )

        self.count = 0
        self.stack_size = self.get_parameter(
            'stack_size').get_parameter_value().integer_value
        self.tactile_stack = np.zeros((self.stack_size, 16), dtype=np.float32)

        self.sub_tactile = self.create_subscription(
            TactileSignal,
            '/tactile_signals',
            self.append_data_callback,
            10
        )

        self.cli = self.create_client(
            ChangeSlidingParameter, 'change_sliding_parameter')
        self.req = ChangeSlidingParameter.Request()

    def append_data_callback(self, msg):
        self.append_data(msg.data)

    def append_data(self, item):
        '''
            Append new data to stack.
            If the stack is full, pop out the first item.
        '''
        if self.count < self.stack_size:
            self.tactile_stack[self.count, :] = item
        else:
            self.tactile_stack[:-1, :] = self.tactile_stack[1:, :]
            self.tactile_stack[-1] = item

        self.count += 1

        if self.count % self.stack_size == 0:
            # Perception process
            cov_matrix = fourier_cov(self.tactile_stack)

            self.send_request(np.random.rand() * 0.1 + 0.05, 1.0, 0.25)
            try:
                res = self.future.result()
                if not res.success:
                    self.get_logger().warn('Sliding parameters not changed')
            except Exception as e:
                self.get_logger().warn('Change sliding parameter service call failed %r' % (e, ))

    def send_request(self, speed, force, distance=0.3):
        self.req.distance = distance
        self.req.force = force
        self.req.speed = speed
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
