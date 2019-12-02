import argparse
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from franka_msgs.msg import FrankaCommand, FrankaState, TactileSignal


# PID parameters
kp = 1e-5
ki = 1e-6
kd = 1e-6
window_size = 15

theta = 0.01


def rotMat(alpha, beta, gamma):
    ''' Compute rotational matrix'''

    rotation_x = np.matrix([[1, 0,              0],
                            [0, np.cos(alpha), -np.sin(alpha)],
                            [0, np.sin(alpha),  np.cos(alpha)]])
    rotation_y = np.matrix([[np.cos(beta), 0, np.sin(beta)],
                            [0,            1, 0],
                            [-np.sin(beta), 0, np.cos(beta)]])
    rotation_z = np.matrix([[np.cos(gamma), -np.sin(gamma), 0],
                            [np.sin(gamma),  np.cos(gamma), 0],
                            [0,              0,             1]])
    return rotation_x * rotation_y * rotation_z


class SignalProcessor(Node):

    def __init__(self, qos_profile):
        super().__init__('signal_processor')
        self.i = 0

        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('Reliable communicator')
        else:
            self.get_logger().info('Best effort communicator')

        # Setup subscriber
        self.sub_bot = self.create_subscription(
            FrankaState, 'robot_states', self.robot_state_callback, qos_profile)
        self.robot_states = np.zeros((window_size, 28), dtype=np.float64)

        self.sub_tac = self.create_subscription(
            TactileSignal, 'tactile_signals', self.tactile_callback, qos_profile)
        self.data = np.zeros((window_size, 16), dtype=np.int)
        self.error = np.zeros(window_size, dtype=np.float64)
        self.error_d = 0.0
        self.error_sum = 0.0
        self.target_val = 5200

        # Setup publisher
        self.timer_period = 0.3
        self.pub = self.create_publisher(
            FrankaCommand, 'franka_commands', qos_profile)
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    # Subscriber callback
    def robot_state_callback(self, msg):
        O_T_EE = msg.o_t_ee
        O_F_EXT_HAT_K = msg.o_f_ext_hat_k
        TAU_EXT_HAT_FILTERED = msg.tau_ext_hat_filtered
        V_EE = msg.v_ee
        self.robot_states[1:, :] = self.robot_states[:-1, :]
        self.robot_states[0, :] = np.concatenate(
            (O_T_EE, O_F_EXT_HAT_K, V_EE), axis=0)

    def tactile_callback(self, msg):
        self.i += 1

        error = np.mean(msg.data) - self.target_val
        self.error_d = error - self.error[0]
        self.error[1:] = self.error[:-1]
        self.error[0] = error
        self.error_sum = np.sum(self.error)

        self.data[1:, :] = self.data[:-1, :]
        self.data[0, :] = msg.data

    # Timer callback for publisher
    def timer_callback(self):
        # x0, y0, z0         = self.robot_states[10], self.robot_states[11], self.robot_states[12]
        # alpha, beta, gamma = self.robot_states[13], self.robot_states[14], self.robot_states[15]
        # rotation_matrix = rotMat(alpha, beta, gamma)
        # dest = rotation_matrix * theta + [[x0], [y0], [z0]]
        if self.i <= 300: 
            x = 0.0
            y = 0.0
        else:
            x = np.sin(self.i / 100) * 0.02
            y = np.sin(self.i / 100) * 0.02

        z = kp * self.error[0] + ki * self.error_sum + kd * self.error_d
        msg = FrankaCommand()
        msg.header.frame_id = 'base'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = [x, y, z, 0.0, 0.0, 0.0]
        msg.response_time = self.timer_period
        self.pub.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='set qos profile to reliable')
    parser.set_defaults(reliable=False)
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(argv)
    rclpy.init(args=args.argv)

    if args.reliable:
        custom_qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    else:
        custom_qos_profile = qos_profile_sensor_data

    node = SignalProcessor(custom_qos_profile)

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
