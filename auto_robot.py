#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoRobot(Node):
    def __init__(self):
        super().__init__("auto_robot")

        # Publish directly to robot (no teleop)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Auto robot node started")
        self.get_logger().info("Waiting 3 seconds before starting...")

        self.has_run = False
        self.start_time = time.time()

        self.timer = self.create_timer(0.1, self.loop)

    def send_cmd(self, lin=0.0, ang=0.0):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def run_motion(self, lin, ang, duration, label):
        self.get_logger().info(label)
        start = time.time()

        while time.time() - start < duration and rclpy.ok():
            self.send_cmd(lin, ang)
            time.sleep(0.1)

        # Stop between motions
        self.send_cmd(0.0, 0.0)
        time.sleep(0.5)

    def run_sequence(self):
        self.get_logger().info("Starting autonomous startup sequence")

        # Forward
        self.run_motion(0.2, 0.0, 1.5, "Forward")

        # Backward
        self.run_motion(-0.2, 0.0, 1.5, "Backward")

        # Turn right to ~45°
        self.run_motion(0.0, -0.4, 1.0, "Turn right (to 45°)")

        # Turn left to ~135°
        self.run_motion(0.0, 0.4, 2.0, "Turn left (to 135°)")

        # Return to ~90°
        self.run_motion(0.0, -0.4, 1.0, "Return to 90°")

        self.get_logger().info("Sequence complete")
        self.send_cmd(0.0, 0.0)

    def loop(self):
        if self.has_run:
            return

        # Wait 3 seconds after startup
        if time.time() - self.start_time < 3.0:
            return

        self.has_run = True
        self.timer.cancel()

        self.run_sequence()


def main(args=None):
    rclpy.init(args=args)
    node = AutoRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
