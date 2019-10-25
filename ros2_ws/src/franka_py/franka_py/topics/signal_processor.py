import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from franka_msgs.msg import FrankaCommand
from franka_msgs.msg import TactileSignal


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
        self.data = np.zeros(16, dtype=np.int)

        # Setup publisher
        timer_period = 1
        self.pub = self.create_publisher(
            FrankaCommand, 'franka_commands', qos_profile)
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    # Subscriber callback
    def tactile_callback(self, msg):
        self.data = msg.pressure

    # Timer callback for publisher
    def timer_callback(self):
        msg = FrankaCommand()
        msg.header.frame_id = 'base'
        # msg.header.stamp = self.get_clock().now()

        target_val = 5100
        mean_vals = np.mean(self.data[0], dtype=np.int)
        z = 0.000 / (self.i + 1) * (mean_vals - target_val)
        msg.command = [0.0, 0.0, z, 0.0, 0.0, 0.0]
        commands = ', '.join([str(c) for c in msg.command])
        self.i += 1
        self.get_logger().info('Sent commands: [%s]' % commands)
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