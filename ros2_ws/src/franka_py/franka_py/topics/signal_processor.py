import argparse
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from franka_msgs.msg import FrankaCommand
from franka_msgs.msg import TactileSignal


# PID parameters
kp = 0.0001
ki = 0.000001
kd = 0.00001
window_size = 15


class SignalProcessor(Node):

    def __init__(self, qos_profile):
        super().__init__('signal_processor')
        self.i = 0

        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('Reliable communicator')
        else:
            self.get_logger().info('Best effort communicator')

        # Setup subscriber
        self.sub = self.create_subscription(
            TactileSignal, 'tactile_signals', self.tactile_callback, qos_profile)
        self.data = np.zeros((window_size, 16), dtype=np.int)
        self.error = np.zeros(window_size, dtype=np.float64)
        self.error_d = 0.0
        self.error_sum = 0.0
        self.target_val = 5200

        # Setup publisher
        timer_period = 1.0
        self.pub = self.create_publisher(
            FrankaCommand, 'franka_commands', qos_profile)
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    # Subscriber callback
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
        msg = FrankaCommand()
        msg.header.frame_id = 'base'
        msg.header.stamp = self.get_clock().now().to_msg()
        y = 0.0
        z = kp * self.error[0] + ki * self.error_sum + kd * self.error_d
        msg.command = [0.0, y, z, 0.0, 0.0, 0.0]
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
