# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
import numpy as np
import rclpy
from rclpy.node import Node

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import ChangeSlidingParameter
from tactile_interfaces.msg import TactileSignal

from finger_sense.Perceptum import Perceptum


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
                ('mode', 'train')
            ]
        )

        save_dir = self.get_parameter(
            'save_dir').get_parameter_value().string_value

        self.mode = self.get_parameter(
            'mode').get_parameter_value().string_value

        self.count = 0
        self.motion_finished = False
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

        self.create_knowledge_base()

        self.prev_control_params = np.zeros((1, 2))
        self.current_control_params = np.array([0.1, 0.01])

    def create_knowledge_base(self):
        # Parse directory
        core_dir = self.get_parameter(
            'core_dir').get_parameter_value().string_value
        factor_dir = self.get_parameter(
            'factor_dir').get_parameter_value().string_value
        info_dir = self.get_parameter(
            'info_dir').get_parameter_value().string_value
        n_basis = self.get_parameter(
            'n_basis').get_parameter_value().integer_value

        # Load knowledge base
        self.perceptum = Perceptum(
            [core_dir, factor_dir, info_dir], n_basis, 16, 'Gaussian')

    def robot_callback(self, msg):
        self.motion_finished = msg.motion_finished
        self.robot_state[0, 0:6] = msg.o_f_ext_hat_k
        self.robot_state[0, 6:9] = msg.position
        self.robot_state[0, 9:13] = msg.quaternion

    def tactile_callback(self, msg):
        item = msg.data

        is_control_updated = False

        if item is not None:
            self.count += 1

            if self.count < self.stack_size:
                self.tactile_stack[self.count, :] = item
            else:
                self.tactile_stack[:-1, :] = self.tactile_stack[1:, :]
                self.tactile_stack[-1] = item

                # Select perception mode
                if self.mode == 'train':
                    self.perceptum.perceive(self.tactile_stack, 'train')

                    if self.motion_finished:
                        force = self.current_control_params[0]
                        speed = self.current_control_params[1]
                        if force <= 1.0:
                            force += 0.1
                        if speed <= 0.1:
                            speed += 0.01
                        else:
                            speed = -1.0
                        self.prev_control_params = self.current_control_params
                        self.current_control_params = np.array([force, speed])

                        is_control_updated = True

                elif self.mode == 'test':
                    if self.motion_finished:
                        gradients, weights, delta_latent = self.perceptum.perceive(
                            self.tactile_stack)
                        new_control = np.sum(weights * np.matmul(gradients, np.outer(delta_latent, 1/(
                            self.current_control_params - self.   prev_control_params))), axis=0)
                        self.prev_control_params = self.current_control_params
                        self.current_control_params = new_control

                        is_control_updated = True

                    if self.count > 960:
                        speed = -1.0

                if is_control_updated:
                    distance = 0.25
                    force = self.current_control_params[0]
                    speed = self.current_control_params[1]
                    try:
                        self.send_request(distance, force, speed)
                    except Exception as e:
                        self.get_logger().warn('Change sliding parameter service call failed %r' % (e, ))

                    is_control_updated = False

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
