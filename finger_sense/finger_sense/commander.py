# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
import numpy as np
import os
import rclpy
import time
from collections import deque
from rclpy.node import Node
from std_msgs.msg import String

from franka_interfaces.msg import RobotState
from franka_interfaces.srv import SlidingControl
from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState


LATENT_DIM = 3
STACK_SIZE = 32

# train params
MATERIAL = "Velvet2Wool"
DISTANCE = 0.18
PARAMS = []
for i in range(2):
    for j in range(2):
        for _ in range(2):
            PARAMS.append((i*1.0+1.0, -j*0.005-0.01, -DISTANCE))
            PARAMS.append((i*1.0+1.0, j*0.005+0.01, DISTANCE))
PARAMS.append((-1.0, 0.0, 0.0))
SAVE_COUNT = 4


class Commander(Node):
    def __init__(self):
        super().__init__("commander")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("save_dir", None),
                ("mode", None),
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
        self.stack = deque(maxlen=None)

        self.send_sliding_control_request(
            0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def get_params(self):
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.mode = str(self.get_parameter("mode").value)

    def robot_state_callback(self, msg):
        states = [msg.position, msg.orientation,
                  msg.velocity, msg.external_wrench]
        self.robot_state = np.hstack(states)

    def tactile_callback(self, msg):
        self.buffer.append(np.hstack([msg.data, self.robot_state]))
        self.stack.append(msg.data)

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
                force = PARAMS[self.count][0]
                dy = PARAMS[self.count][1]
                y = PARAMS[self.count][2]
                self.send_sliding_control_request(
                    force, [0.0, y, 0.0], [0.0, dy, 0.0])

                # Save buffer
                if control_type == 3 and (self.count + 1) % SAVE_COUNT == 0:
                    basename="{}_{:.1f}N_{:.3f}mmps_{}.npy".format(
                        MATERIAL,
                        PARAMS[self.count-1][0],
                        abs(PARAMS[self.count-1][1]),
                        nanoseconds)
                    filename=os.path.join(self.save_dir, MATERIAL, basename)
                    np.save(filename, self.buffer)
                    self.buffer.clear()
                    self.get_logger().info("Saved to file {}.".format(filename))

                self.count += 1

            # Save whole stack
            elif self.mode == "train" and self.count >= len(PARAMS):
                basename="{}_{}.npy".format(MATERIAL, nanoseconds)
                filename=os.path.join(self.save_dir, MATERIAL, basename)
                np.save(filename, self.stack)
                self.get_logger().info("Saved to file {}.".format(filename))
                self.destroy_node()

    def send_sliding_control_request(self, force, distance, speed):
        """
        Send parameter change request to control parameter server
        """
        self.sliding_control_req.force=force
        self.sliding_control_req.distance=distance
        self.sliding_control_req.speed=speed
        self.sliding_control_future=self.sliding_control_cli.call_async(
            self.sliding_control_req
        )

    def send_sensor_request(self, transition):
        self.sensor_req.transition=transition
        self.sensor_future=self.sensor_cli.call_async(self.sensor_req)


def main(args = None):
    time.sleep(2)
    rclpy.init(args = args)
    node=Commander()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
