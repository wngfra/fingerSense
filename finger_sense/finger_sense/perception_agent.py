# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy
from mpl_toolkits.mplot3d import Axes3D
from rclpy.node import Node
from skfda.representation.basis import Fourier

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import ChangeSlidingParameter
from tactile_interfaces.msg import TactileSignal

from finger_sense.utility import basis_expand, error_ellipsoid, KL_divergence_normal, project2vec


class PerceptionAgent(Node):

    def __init__(self):
        super().__init__('perception_agent')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('core_dir', './src/fingerSense/finger_sense/finger_sense/core.npy'),
                ('factor_dir', './src/fingerSense/finger_sense/finger_sense/factors.npy'),
                ('info_dir', './src/fingerSense/finger_sense/finger_sense/info.csv'),
                ('save_dir', './src/fingerSense/finger_sense/finger_sense/data.npy'),
                ('n_basis', 33),
                ('stack_size', 64),
            ]
        )

        save_dir = self.get_parameter(
            'save_dir').get_parameter_value().string_value
        n_basis = self.get_parameter(
            'n_basis').get_parameter_value().integer_value

        self.count = 0
        self.fda_basis = Fourier([0, 2 * np.pi], n_basis=n_basis, period=1)
        self.robot_state = np.zeros((1, 13), dtype=np.float64)
        self.save_dir = save_dir
        self.stack_size = self.get_parameter(
            'stack_size').get_parameter_value().integer_value
        self.tactile_stack = np.zeros((self.stack_size, 16), dtype=np.float32)

        self.sub_robot = self.create_subscription(
            RobotState,
            '/franka_state',
            self.robot_callback,
            100
        )
        self.sub_tactile = self.create_subscription(
            TactileSignal,
            '/tactile_signals',
            self.tactile_callback,
            10
        )

        self.cli = self.create_client(
            ChangeSlidingParameter, 'change_sliding_parameter')
        self.req = ChangeSlidingParameter.Request()

        self.percepts = []
        self.known_percepts = {}
        self.create_knowledge_base()

        # fig = plt.figure(figsize=(8, 6), dpi=80)
        # self.ax1 = fig.add_subplot(111)

    def create_knowledge_base(self):
        # Knowledge base directory
        core_dir = self.get_parameter(
            'core_dir').get_parameter_value().string_value
        factor_dir = self.get_parameter(
            'factor_dir').get_parameter_value().string_value
        info_dir = self.get_parameter(
            'info_dir').get_parameter_value().string_value

        # Load knowledge base
        self.core = np.load(core_dir, allow_pickle=True).squeeze()
        self.factors = np.load(factor_dir, allow_pickle=True)[0:2]
        self.info = pd.read_csv(info_dir, delimiter=',')
        class_names = self.info['class_name']

        for unique_class in set(class_names):
            data = self.core[:, unique_class == class_names]
            mean = np.mean(data, axis=1)
            cov = np.cov(data)
            self.known_percepts[unique_class] = [mean, cov]

    def robot_callback(self, msg):
        self.robot_state[0, 0:6] = msg.o_f_ext_hat_k
        self.robot_state[0, 6:9] = msg.position
        self.robot_state[0, 9:13] = msg.quaternion

    def tactile_callback(self, msg):
        self.append_data(msg.data)

    def append_data(self, item):
        '''
            Append new data to stack.
            If the stack is full, pop out the first item.
        '''
        if item is not None:
            self.count += 1

            if self.count < self.stack_size:
                self.tactile_stack[self.count, :] = item
            else:
                self.tactile_stack[:-1, :] = self.tactile_stack[1:, :]
                self.tactile_stack[-1] = item
                coeff_cov = basis_expand(self.tactile_stack, self.fda_basis)
                coeff_cov = basis_expand(self.tactile_stack, self.fda_basis)
                percept = project2vec(coeff_cov, self.factors).reshape(1, -1)
                self.percepts.append(percept)
                p_data = np.vstack(self.percepts)
                p = [np.mean(p_data, axis=0), np.cov(p_data.transpose())]
                kl_divs = []
                for key in self.known_percepts.keys():
                    q = self.known_percepts[key]
                    kl_div = KL_divergence_normal(p, q)
                    if np.isnan(kl_div):
                        kl_div = -1
                    kl_divs.append(kl_div)

                objects = self.known_percepts.keys()
                y_pos = np.arange(len(objects))
                plt.cla()
                plt.bar(y_pos, kl_divs, align='center', alpha=0.5)
                plt.xticks(y_pos, objects)
                plt.ylabel('KL divergance')
                plt.title('Perception')
                plt.pause(0.5)
                # self.ax1.imshow(coeff_cov)
                # plt.pause(0.5)
                # self.ax1.scatter(latent_vector[0, 0], latent_vector[0, 1], latent_vector[0, 2])
                # with open(self.save_dir, 'ba+') as f:
                #   vectorized_data = np.hstack([latent_vector, self.robot_state]).reshape(1, -1)
                #   np.savetxt(f, coeff_cov, delimiter=',', fmt='%1.3e')

        if self.count % self.stack_size == 0:
            distance, force, speed = 0.25, 2.0, 0.1 * np.random.rand()
            if self.count >= 960:
                speed = 0.0
                self.get_logger().info('Motion finished.')
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
