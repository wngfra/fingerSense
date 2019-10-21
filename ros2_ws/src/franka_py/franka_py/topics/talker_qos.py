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


class TalkerQos(Node):

    def __init__(self, qos_profile):
        super().__init__('talker_qos')
        self.i = 0
        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('Reliable communicator')
        else:
            self.get_logger().info('Best effort communicator')
        self.pub = self.create_publisher(FrankaCommand, 'franka_commands', qos_profile)
        self.sub = self.create_subscription(
            TactileSignal, 'tactile_signals', self.tactile_callback, qos_profile)

        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def tactile_callback(self, msg):
        pressure_str = [str(p) for p in msg.pressure]
        self.get_logger().info('Received tactile signals: [%s]' % ', '.join(pressure_str))

    def timer_callback(self):
        msg = FrankaCommand()
        msg.header.frame_id = 'base'
        print(msg.header)
        time = self.get_clock().now()
        # testing
        msg.command = np.ones(6) * 0.01
        commands = ', '.join([str(c) for c in msg.command])
        self.i += 1
        self.get_logger().info(commands)
        self.pub.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
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

    node = TalkerQos(custom_qos_profile)

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
