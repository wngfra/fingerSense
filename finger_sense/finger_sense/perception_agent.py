# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
import itertools
import numpy as np
import os.path
import rclpy
from collections import deque
from rclpy.node import Node

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import SlidingControl
from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState

from finger_sense.Perceptum import Perceptum

DISTANCE = 0.1
LATENT_DIM = 3
MAX_COUNT = 1000
NUM_BASIS = 33
STACK_SIZE = 64

MATERIAL_ = 'PLAVertical_'
FORCES = [0.0, 0.0] #[(i + 20.0, -1.0) for i in range(10)]
# FORCES = list(itertools.chain(*FORCES))
SPEEDS = [0.01*j + 0.001 for j in range(5, 0, -1)]


class PerceptionAgent(Node):

    def __init__(self):
        super().__init__('perception_agent')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('core_dir',   None),
                ('factor_dir', None),
                ('save_dir',   None),
                ('mode',       None)
            ]
        )

        self.get_params()

        self.count = 0
        self.robot_state = [np.zeros(6), np.zeros(16)]
        self.tactile_stack = deque(maxlen=STACK_SIZE)

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

        self.prev_control_params = np.zeros(4)
        self.current_control_params = np.zeros(4)

        # Create a perceptum class
        self.perceptum = Perceptum(
            self.dirs,
            LATENT_DIM,  # latent dimension
            NUM_BASIS,  # number of basis
            'Gaussian'
        )

        self.direction = 1.0

        self.lap = 0
        self.index = [0, 0]

        self.trainset = []

    def get_params(self):
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.mode = str(self.get_parameter('mode').value)

        core_dir = str(self.get_parameter('core_dir').value)
        factor_dir = str(self.get_parameter('factor_dir').value)

        self.dirs = {
            'core_dir':   core_dir,
            'factor_dir': factor_dir
        }

        # If file not exists, set dir to None
        for k, d in self.dirs.items():
            if not os.path.exists(d):
                self.dirs[k] = None

    def robot_callback(self, msg):
        self.robot_state = [msg.o_f_ext_hat_k, msg.o_t_ee]

    def tactile_callback(self, msg):
        raw_data = msg.data
        if raw_data is None:
            self.get_logger().warn("Tactile data is None-Type.")
        else:
            self.count += 1
            self.tactile_stack.append(raw_data)

            if self.mode == 'train':
                # training mode
                if self.index[0] < len(FORCES):
                    self.trainset.append(raw_data)

                    if self.index[1] >= len(SPEEDS):
                        self.index[0] += 1
                        self.index[1] = 0
                    else:
                        try:
                            response = self.sliding_control_future.result()
                            success = response.success
                            recovered = response.recovered
                        except Exception:
                            success = False
                            recovered = False

                        if self.index[0] == 0 and self.lap == 0 or success or recovered:
                            force = FORCES[self.index[0]]
                            dx = SPEEDS[self.index[1]] * self.direction
                            x = DISTANCE * self.direction

                            t = self.get_clock().now().nanoseconds
                            self.get_logger().info('current time: {}'.format(t))
                            self.send_sliding_control_request(
                                force, [2 * x, x, 0.0], [2.0 * dx, dx, 0.0])

                            self.direction *= -1.0
                            if force >= 0.0:
                                self.lap += 1
                                if self.lap > 3:
                                    self.index[1] += 1
                                    self.lap = 0
                            else:
                                self.index[1] = len(SPEEDS) + 1               

                                # trainset = np.asarray(self.trainset)
                                # filename = self.save_dir + MATERIAL_ + ('%.2f' % force) + '_' + ('%.4f' % dx) + '.csv'
                                # np.savetxt(filename, trainset, delimiter=',', fmt='%d')
                                # self.trainset = []
            else:
                '''FIXME perception mode; update control params
                    using 3D speed and distance vector
                    invert sling direction every loop
                '''

                is_control_updated = False

                try:
                    response = self.sliding_control_future.result()
                    success = response.success
                    recovered = response.recovered
                except Exception:
                    success = False
                    recovered = False

                if success or recovered:
                    gradients, weights, delta_latent = self.perceptum.perceive(
                        self.tactile_stack)
                    new_control = np.sum(weights * np.matmul(gradients, np.outer(delta_latent, 1/(
                        self.current_control_params - self.prev_control_params))), axis=0)
                    self.prev_control_params = self.current_control_params
                    self.current_control_params = new_control
                    is_control_updated = True
                if self.count > MAX_COUNT:
                    speed = np.zeros(3)

                if is_control_updated:
                    # FIXME update control params
                    force = self.current_control_params[0]
                    speed = self.current_control_params[1:]
                    try:
                        self.send_request(force, speed)
                    except Exception as e:
                        self.get_logger().warn('Change sliding parameter service call failed %r' % (e, ))
                    is_control_updated = False

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
