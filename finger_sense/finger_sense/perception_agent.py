# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
import numpy as np
import rclpy
from rclpy.node import Node

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import SlidingControl
from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState

from finger_sense.Perceptum import Perceptum

LATENT_DIM = 3
NUM_BASIS = 33
STACK_SIZE = 64


class PerceptionAgent(Node):

    def __init__(self):
        super().__init__('perception_agent')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('core_dir',   None),
                ('factor_dir', None),
                ('info_dir',   None),
                ('save_dir',   None),
                ('mode',       None)
            ]
        )

        self.get_params()

        self.count = 0
        self.robot_state = np.zeros((1, 13), dtype=np.float64)
        self.tactile_stack = np.zeros((STACK_SIZE, 16), dtype=np.float32)

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

        self.sliding_control_cli = self.create_client(
            SlidingControl, 'sliding_control')
        self.sliding_control_req = SlidingControl.Request()
        self.sensor_cli = self.create_client(
            ChangeState, 'tactile_publisher/change_state')
        self.sensor_req = ChangeState.Request()

        self.prev_control_params = np.zeros(2)
        self.current_control_params = np.zeros(2)

        # Create a perceptum class
        self.perceptum = Perceptum(
            [
                self.core_dir,
                self.factor_dir,
                self.info_dir
            ],
            LATENT_DIM,  # latent dimension
            NUM_BASIS,  # number of basis
            STACK_SIZE,  # stack size
            'Gaussian'
        )

        self.direction = 1.0

        self.lap = 0
        self.index = 0

        self.speeds = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1]
        self.force = 0.1
        self.trainset = []

    def get_params(self):
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.core_dir = str(self.get_parameter('core_dir').value)
        self.factor_dir = str(self.get_parameter('factor_dir').value)
        self.info_dir = str(self.get_parameter('info_dir').value)
        self.mode = str(self.get_parameter('mode').value)

    def robot_callback(self, msg):
        self.robot_state[0, 0:6] = msg.o_f_ext_hat_k
        self.robot_state[0, 6:9] = msg.position
        self.robot_state[0, 9:13] = msg.quaternion

    def tactile_callback(self, msg):
        raw_data = msg.data
        if raw_data is not None:
            self.count += 1
            if self.count < STACK_SIZE:
                self.tactile_stack[self.count] = raw_data
            else:
                self.tactile_stack[:-1, :] = self.tactile_stack[1:, :]
                self.tactile_stack[-1, :] = raw_data

        # training
        self.trainset.append(raw_data)
        if self.index < len(self.speeds):
            x = 0.26 * self.direction
            dx = self.speeds[self.index] * self.direction
                       
            try:
                response = self.sliding_control_future.result()
                success = response.success
            except Exception:
                success = False
                
            if self.index == 0 and self.lap == 0 or success:
                    self.send_sliding_control_request(self.force, [x, 0.0, 0.0], [dx, 0.0, 0.0])
                    self.lap += 1
                    self.direction *= -1.0

                    if self.lap > 5:
                        self.index += 1
                        self.lap = 0

                        trainset = np.asarray(self.trainset)
                        filename = self.save_dir + 'BlackWool_' + str(self.force) + '_' + str(dx) + '.csv'
                        np.savetxt(filename, trainset, delimiter=',', fmt='%d')
                        self.trainset = []

        '''
        is_control_updated = False

        if item is not None:
            self.count += 1

            if self.count < self.stack_size:
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
        '''

    def send_sliding_control_request(self, force, distance, speed):
        '''
            Send parameter change request to control parameter server
        '''
        self.sliding_control_req.force = force
        self.sliding_control_req.distance = distance
        self.sliding_control_req.speed = speed
        self.sliding_control_future = self.sliding_control_cli.call_async(
            self.sliding_control_req)

    def send_sensor_request(self, transition):
        self.sensor_req.transition = transition
        self.sensor_future = self.sensor_cli.call_async(self.sensor_req)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
