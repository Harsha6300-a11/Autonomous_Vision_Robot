#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from vision_person_follower.msg import TrackedPerson


class ControllerNode(Node):

    def __init__(self):
        super().__init__('person_controller')

        self.create_subscription(
            TrackedPerson,
            '/tracked_person',
            self.person_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # LIDAR
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        # FOLLOW PARAMETERS
        self.SOCIAL_MIN = 1.0
        self.SOCIAL_MAX = 1.5
        self.CENTER_TOL = 0.1

        # SEARCH FSM
        self.search_state = 'rotate'
        self.rotation_done = 0.0
        self.last_time = time.time()
        self.move_start = 0.0

        self.get_logger().info('Controller node started')

    # ================= LIDAR =================
    def scan_callback(self, msg):
        r = np.array(msg.ranges)
        c = len(r) // 2

        def clean(x):
            x = x[np.isfinite(x)]
            return np.min(x) if len(x) else float('inf')

        self.front = clean(r[c-10:c+10])
        self.left = clean(r[c+10:c+30])
        self.right = clean(r[c-30:c-10])

    # ================= PERSON =================
    def person_callback(self, msg):
        cmd = Twist()

        if msg.visible:
            # Reset search
            self.search_state = 'rotate'
            self.rotation_done = 0.0

            # Distance control
            if msg.depth < self.SOCIAL_MIN:
                cmd.linear.x = -0.05
            elif msg.depth > self.SOCIAL_MAX:
                cmd.linear.x = 0.12

            # Centering
            if abs(msg.offset) > self.CENTER_TOL:
                cmd.angular.z = -0.35 * msg.offset

        else:
            self.search(cmd)

        self.avoid_obstacle(cmd)
        self.cmd_pub.publish(cmd)

    # ================= SEARCH FSM =================
    def search(self, cmd):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if self.search_state == 'rotate':
            cmd.angular.z = -0.5
            self.rotation_done += 0.5 * dt

            if self.rotation_done >= 2 * np.pi:
                self.rotation_done = 0.0
                self.search_state = 'move'
                self.move_start = now

        elif self.search_state == 'move':
            if now - self.move_start < 1.2:
                cmd.linear.x = 0.12
            else:
                self.search_state = 'rotate'

    # ================= OBSTACLE AVOIDANCE =================
    def avoid_obstacle(self, cmd):
        if self.front < 0.6:
            cmd.linear.x = 0.05
            cmd.angular.z = 0.6 if self.left > self.right else -0.6


def main():
    rclpy.init()
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
