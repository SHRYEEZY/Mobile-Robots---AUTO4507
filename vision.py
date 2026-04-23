#!/usr/bin/env python3

import math
import os
from datetime import datetime

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, Imu


class FrontObjectPhotoCapture(Node):
    def __init__(self):
        super().__init__('front_object_photo_capture')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('save_dir', os.path.expanduser('~/captured_objects'))
        self.declare_parameter('front_angle_deg', 12.0)
        self.declare_parameter('trigger_distance_m', 1.8)
        self.declare_parameter('min_valid_distance_m', 0.15)
        self.declare_parameter('max_yaw_rate_rad_s', 0.15)
        self.declare_parameter('cooldown_sec', 3.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.min_valid_distance_m = float(self.get_parameter('min_valid_distance_m').value)
        self.max_yaw_rate_rad_s = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_front_distance = None
        self.latest_yaw_rate = 0.0
        self.last_capture_time = self.get_clock().now()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10
        )

        self.timer = self.create_timer(0.2, self.check_and_capture)

        self.get_logger().info('Front object photo capture node started')
        self.get_logger().info(f'Image topic: {self.image_topic}')
        self.get_logger().info(f'Scan topic:  {self.scan_topic}')
        self.get_logger().info(f'IMU topic:   {self.imu_topic}')
        self.get_logger().info(f'Saving images to: {self.save_dir}')

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def imu_callback(self, msg: Imu):
        # Use z angular velocity as a simple "am I turning too much?" check
        self.latest_yaw_rate = float(msg.angular_velocity.z)

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.latest_front_distance = None
            return

        front_half_angle_rad = math.radians(self.front_angle_deg)

        valid_distances = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle) <= front_half_angle_rad:
                if self.min_valid_distance_m <= r <= msg.range_max:
                    valid_distances.append(r)

        if valid_distances:
            self.latest_front_distance = min(valid_distances)
        else:
            self.latest_front_distance = None

    def check_and_capture(self):
        if self.latest_frame is None:
            return

        if self.latest_front_distance is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_capture_time).nanoseconds / 1e9

        object_ahead = self.latest_front_distance <= self.trigger_distance_m
        robot_stable = abs(self.latest_yaw_rate) <= self.max_yaw_rate_rad_s
        cooldown_ok = elapsed >= self.cooldown_sec

        if object_ahead and robot_stable and cooldown_ok:
            self.save_capture()
            self.last_capture_time = now

    def save_capture(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        image_path = os.path.join(self.save_dir, f'object_{timestamp}.jpg')
        meta_path = os.path.join(self.save_dir, f'object_{timestamp}.txt')

        frame = self.latest_frame.copy()

        text1 = f'Front distance: {self.latest_front_distance:.2f} m'
        text2 = f'Yaw rate z: {self.latest_yaw_rate:.3f} rad/s'

        cv2.putText(
            frame, text1, (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
        )
        cv2.putText(
            frame, text2, (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
        )

        ok = cv2.imwrite(image_path, frame)

        if not ok:
            self.get_logger().error(f'Failed to save image: {image_path}')
            return

        with open(meta_path, 'w', encoding='utf-8') as f:
            f.write(f'timestamp: {timestamp}\n')
            f.write(f'front_distance_m: {self.latest_front_distance:.3f}\n')
            f.write(f'yaw_rate_z_rad_s: {self.latest_yaw_rate:.6f}\n')
            f.write(f'image_path: {image_path}\n')

        self.get_logger().info(
            f'Captured object photo -> {image_path} '
            f'(distance={self.latest_front_distance:.2f} m, '
            f'yaw_rate={self.latest_yaw_rate:.3f} rad/s)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontObjectPhotoCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()