import numpy as np

import rclpy
from rclpy.node import Node

from franka_msgs.msg import TactileSignal

class Cellebrum(Node):
    
    def __init__(self):
        super().__init__('cellebrum')
        self.sub = self.create_subscription(TactileSignal, 'tactile_signals', self.sub_callback, 10)

    def sub_callback(self, msg):
        self.get_logger().info('Received [%f]' % msg.pressure)


def main(args=None):
    rclpy.init(args=args)

    node = Cellebrum()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()