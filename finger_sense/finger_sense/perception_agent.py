# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from mpl_toolkits.mplot3d import Axes3D
from rclpy.node import Node
from skfda.representation.basis import Fourier

from franka_interfaces.srv import ChangeSlidingParameter
from tactile_interfaces.msg import TactileSignal

from finger_sense.percepy import basis_expand, project2vec


class PerceptionAgent(Node):

    def __init__(self):
        super().__init__('perception_agent')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('core_dir', './src/fingerSense/finger_sense/finger_sense/core.npy'),
                ('factor_dir', './src/fingerSense/finger_sense/finger_sense/factors.npy'),
                ('n_basis', 33),
                ('stack_size', 128),
            ]
        )

        # Knowledge base directory
        core_dir = self.get_parameter(
            'core_dir').get_parameter_value().string_value
        factor_dir = self.get_parameter(
            'factor_dir').get_parameter_value().string_value
        n_basis = self.get_parameter(
            'n_basis').get_parameter_value().integer_value

        self.fda_basis = Fourier([0, 2 * np.pi], n_basis=n_basis, period=1)
        self.count = 0
        self.core = np.load(core_dir, allow_pickle=True)
        self.factors = np.load(factor_dir, allow_pickle=True)
        self.stack_size = self.get_parameter(
            'stack_size').get_parameter_value().integer_value
        self.tactile_stack = np.zeros((self.stack_size, 16), dtype=np.float32)
        self.vec_list = []

        self.sub_tactile = self.create_subscription(
            TactileSignal,
            '/tactile_signals',
            self.append_data_callback,
            10
        )

        self.cli = self.create_client(
            ChangeSlidingParameter, 'change_sliding_parameter')
        self.req = ChangeSlidingParameter.Request()

        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')

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
            coeff_cov = basis_expand(self.tactile_stack, self.fda_basis)
            self.vec_list.append(project2vec(coeff_cov, self.factors))
            vec_array = np.array(self.vec_list).reshape(-1, 3)
            self.ax.scatter(vec_array[:, 0], vec_array[:, 1], vec_array[:, 2])

        plt.draw()
        plt.pause(0.001)
        self.count += 1

        if self.count % self.stack_size == 0:
            distance, force, speed = 0.25, 0.5, 0.1 * np.random.rand()
            if self.count >= 320:
                speed = 0.0
            try:
                self.send_request(distance, force, speed)
            except Exception as e:
                self.get_logger().warn('Change sliding parameter service call failed %r' % (e, ))

    def send_request(self, distance=0.3, force=0.0, speed=0.0):
        '''
            Send parameter change request to control parameter server
        '''
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
