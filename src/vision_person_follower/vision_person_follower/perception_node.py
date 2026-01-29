#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

from vision_person_follower.msg import TrackedPerson


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('person_perception')

        self.bridge = CvBridge()
        self.detector = YOLO("yolov8n.pt")
        self.tracker = DeepSort(max_age=30)

        self.target_id = None
        self.last_seen = 0.0
        self.TIMEOUT = 2.0

        self.PERSON_HEIGHT = 1.7
        self.FOCAL = 1800.0

        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            TrackedPerson,
            '/tracked_person',
            10
        )

        self.get_logger().info('Perception node started')

    def image_callback(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        detections = []
        results = self.detector(frame, verbose=False)

        for r in results:
            for b in r.boxes:
                if self.detector.names[int(b.cls[0])] == 'person':
                    x1, y1, x2, y2 = map(int, b.xyxy[0])
                    detections.append(
                        ([x1, y1, x2 - x1, y2 - y1],
                         float(b.conf[0]),
                         'person')
                    )

        tracks = self.tracker.update_tracks(detections, frame=frame)

        msg_out = TrackedPerson()
        msg_out.visible = False

        now = time.time()

        for t in tracks:
            if t.is_confirmed():
                l, t_, r, b = map(int, t.to_ltrb())
                box_h = b - t_
                depth = (self.PERSON_HEIGHT * self.FOCAL) / max(box_h, 1)

                w = frame.shape[1]
                offset = ((l + r) / 2 - w / 2) / (w / 2)

                self.target_id = t.track_id
                self.last_seen = now

                msg_out.id = t.track_id
                msg_out.depth = float(depth)
                msg_out.offset = float(offset)
                msg_out.visible = True
                break

        if self.target_id and (now - self.last_seen > self.TIMEOUT):
            self.target_id = None

        self.publisher.publish(msg_out)


def main():
    rclpy.init()
    rclpy.spin(PerceptionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

