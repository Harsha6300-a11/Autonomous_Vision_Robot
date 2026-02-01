#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort


class SmartSocialFollower(Node):

    def __init__(self):
        super().__init__('smart_social_follower')

        # ================= MODELS =================
        self.model = YOLO("yolov8n.pt")
        self.tracker = DeepSort(max_age=30)
        self.bridge = CvBridge()

        # ================= ROS ====================
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/tracked_image', 10)

        # ================= LIDAR =================
        self.front_center = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')

        # ================= TRACKING =================
        self.target_id = None
        self.target_last_seen = 0.0
        self.TARGET_TIMEOUT = 2.0

        # ================= DEPTH =================
        self.FOCAL_CONST = 1800.0
        self.PERSON_HEIGHT_M = 1.7

        self.SOCIAL_MIN = 1.0
        self.SOCIAL_MAX = 1.5
        self.SOCIAL_RELEASE = 1.7

        self.CENTER_TOL = 0.1
        self.MAX_BOX_RATIO = 0.55

        # ================= SEARCH FSM =================
        self.search_state = "rotate"
        self.rotation_done = 0.0
        self.move_start = 0.0

        self.ROT_SPEED = 0.5
        self.FWD_SPEED = 0.12
        self.FWD_TIME = 1.2

        self.last_time = time.time()

        self.get_logger().info("✅ Deterministic 360° Search + Smart Obstacle Avoidance")

    # =====================================================
    # LIDAR
    # =====================================================
    def scan_callback(self, msg):
        r = np.array(msg.ranges)
        n = len(r)
        c = n // 2

        def clean(x):
            x = x[np.isfinite(x)]
            return np.min(x) if len(x) else float('inf')

        d15 = int(n * 15 / 360)
        d35 = int(n * 35 / 360)

        self.front_center = clean(r[c-d15:c+d15])
        self.front_left = clean(r[c+d15:c+d35])
        self.front_right = clean(r[c-d35:c-d15])

    # =====================================================
    # IMAGE CALLBACK
    # =====================================================
    def image_callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        cmd = Twist()

        target = self.detect_target(frame)

        # ---------- PERSON FOUND ----------
        if target:
            self.search_state = "rotate"
            self.rotation_done = 0.0
            self.follow(target, frame, cmd)
            self.avoid_obstacle(cmd)
            self.publish(cmd, frame)
            return

        # ---------- SEARCH ----------
        self.search(cmd)
        self.avoid_obstacle(cmd)
        self.publish(cmd, frame)

    # =====================================================
    # DETECT TARGET
    # =====================================================
    def detect_target(self, frame):
        now = time.time()
        results = self.model(frame, verbose=False)
        detections = []

        for r in results:
            for b in r.boxes:
                if self.model.names[int(b.cls[0])] == "person":
                    x1, y1, x2, y2 = map(int, b.xyxy[0])
                    detections.append(([x1, y1, x2-x1, y2-y1], float(b.conf[0]), 'person'))

        tracks = self.tracker.update_tracks(detections, frame=frame)

        for t in tracks:
            if t.is_confirmed():
                self.target_id = t.track_id
                self.target_last_seen = now
                return t

        if self.target_id and now - self.target_last_seen > self.TARGET_TIMEOUT:
            self.target_id = None

        return None

    # =====================================================
    # FOLLOW (UNCHANGED)
    # =====================================================
    def follow(self, target, frame, cmd):
        h, w, _ = frame.shape
        l, t, r, b = map(int, target.to_ltrb())

        box_h = b - t
        depth = (self.PERSON_HEIGHT_M * self.FOCAL_CONST) / max(box_h, 1)
        offset = ((l + r)/2 - w/2) / (w/2)
        box_ratio = box_h / h

        if self.SOCIAL_MIN <= depth <= self.SOCIAL_MAX:
            return

        if depth < self.SOCIAL_MIN or box_ratio > self.MAX_BOX_RATIO:
            cmd.linear.x = -0.05
            return

        if depth > self.SOCIAL_MAX:
            cmd.linear.x = 0.12

        if abs(offset) > self.CENTER_TOL:
            cmd.angular.z = -0.35 * offset

        cv2.rectangle(frame, (l, t), (r, b), (0, 255, 0), 2)
        cv2.putText(frame, f"ID {target.track_id} {depth:.2f}m",
                    (l, t-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # =====================================================
    # SEARCH (360° → MOVE → REPEAT)
    # =====================================================
    def search(self, cmd):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if self.search_state == "rotate":
            cmd.angular.z = -self.ROT_SPEED
            self.rotation_done += abs(self.ROT_SPEED) * dt

            if self.rotation_done >= 2 * np.pi:
                self.rotation_done = 0.0
                self.search_state = "move"
                self.move_start = now

        elif self.search_state == "move":
            if now - self.move_start < self.FWD_TIME:
                cmd.linear.x = self.FWD_SPEED
            else:
                self.search_state = "rotate"

    # =====================================================
    # OBSTACLE AVOIDANCE (NO STOPPING)
    # =====================================================
    def avoid_obstacle(self, cmd):
        if self.front_center < 0.6:
            if self.front_left > self.front_right:
                cmd.angular.z = 0.6
            else:
                cmd.angular.z = -0.6
            cmd.linear.x = 0.05

    # =====================================================
    # PUBLISH
    # =====================================================
    def publish(self, cmd, frame):
        self.cmd_pub.publish(cmd)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def main():
    rclpy.init()
    node = SmartSocialFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
