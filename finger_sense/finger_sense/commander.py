# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
from typing import final
import numpy as np
import os
import rclpy
import time
from collections import deque
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import SlidingControl
from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState


# train params
MATERIAL = "BeigeLinen"
DISTANCE = 0.15
PARAMS = []
for i in range(1):
    for j in range(5):
        PARAMS.append((i*1.0+8.0, -j*0.005-0.01, -DISTANCE))
        PARAMS.append((i*1.0+8.0,  j*0.005+0.01,  DISTANCE))
PARAMS.append((-1.0, 0.0, 0.0))


class Commander(Node):
    def __init__(self):
        super().__init__("commander")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("save_dir", ''),
                ("mode", ''),
            ],
        )
        self.get_params()

        self.pub = self.create_publisher(String, 'commander_state', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.sub_robot = self.create_subscription(
            RobotState, "franka_state", self.robot_state_callback, 100
        )
        self.sub_tactile = self.create_subscription(
            TactileSignal, "tactile_signals", self.tactile_callback, 10
        )
        self.sliding_control_cli = self.create_client(
            SlidingControl, "sliding_control")
        self.sliding_control_req = SlidingControl.Request()
        self.sensor_cli = self.create_client(
            ChangeState, "tactile_publisher/change_state"
        )
        self.sensor_req = ChangeState.Request()

        # control params
        self.direction = -1.0
        self.robot_state = np.zeros(19)
        self.record = False
        # mode dependent params
        if self.mode == "train":
            self.count = 0
            self.initialized = False
        self.buffer = deque(maxlen=None)

        self.send_sliding_control_request(
            0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def get_params(self):
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.mode = str(self.get_parameter("mode").value)

    def robot_state_callback(self, msg):
        self.robot_state = np.hstack([msg.position, msg.external_wrench])

    def tactile_callback(self, msg):
        self.buffer.append(msg.data)

    def timer_callback(self):
        success = False
        control_type = -1
        try:
            response = self.sliding_control_future.result()
            success = response.success
            control_type = response.type
        except Exception:
            success = False
            control_type = -1

        nanoseconds = self.get_clock().now().nanoseconds

        if success:
            if self.mode == "train" and self.count < len(PARAMS):
                # Save buffer
                if control_type == 3:
                    basename = "{}_{:.1f}N_{:.3f}mps_{}".format(
                        MATERIAL,
                        PARAMS[self.count-1][0],
                        PARAMS[self.count-1][1],
                        nanoseconds)
                    filename = os.path.join(self.save_dir, basename)
                    np.save(filename, self.buffer)
                    self.get_logger().info("Saved to file {}.npy".format(filename))

                self.buffer.clear()
                force = PARAMS[self.count][0]
                dy = PARAMS[self.count][1]
                y = PARAMS[self.count][2]
                self.send_sliding_control_request(
                    force, [0.0, y, 0.0], [0.0, dy, 0.0])

                self.count += 1

    def send_sliding_control_request(self, force, distance, speed):
        """
        Send parameter change request to control parameter server
        """
        self.sliding_control_req.force = force
        self.sliding_control_req.distance = distance
        self.sliding_control_req.speed = speed
        self.sliding_control_future = self.sliding_control_cli.call_async(
            self.sliding_control_req
        )

    def send_sensor_request(self, transition):
        self.sensor_req.transition = transition
        self.sensor_future = self.sensor_cli.call_async(self.sensor_req)


def main(args=None):
    time.sleep(3)
    rclpy.init(args=args)
    node = Commander()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
