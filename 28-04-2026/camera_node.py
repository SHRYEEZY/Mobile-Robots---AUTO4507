#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera_node')

        self.declare_parameter('save_dir', '/output')
        self.declare_parameter('publish_rate', 1.0)

        self.save_dir = self.get_parameter('save_dir').value
        self.rate_hz  = self.get_parameter('publish_rate').value
        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/oak/rgb/image_raw', 10)

        # Build depthai v3 pipeline — Device is managed internally by pipeline
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        video_out = cam.requestOutput((1920, 1080), type=dai.ImgFrame.Type.BGR888p)
        self.q_rgb = video_out.createOutputQueue()
        self.pipeline.start()

        self.timer = self.create_timer(1.0 / self.rate_hz, self.capture_and_publish)

        self.get_logger().info(
            f'OAK camera node ready — publishing on /oak/rgb/image_raw @ {self.rate_hz} Hz'
        )

    def capture_and_publish(self):
        try:
            in_rgb = self.q_rgb.get()
            frame  = in_rgb.getCvFrame()
        except Exception as e:
            self.get_logger().error(f'Frame capture failed: {e}')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_camera'
        self.publisher_.publish(msg)
        self.get_logger().info('Published frame to /oak/rgb/image_raw')

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename  = os.path.join(self.save_dir, f'oak_{timestamp}.jpg')
        cv2.imwrite(filename, frame)
        self.get_logger().info(f'Saved: {filename}')

    def destroy_node(self):
        self.pipeline.stop()   # v3 cleanup — no device.close() needed
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()