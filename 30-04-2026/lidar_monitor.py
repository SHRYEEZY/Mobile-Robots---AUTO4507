#!/usr/bin/env python3
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

OBSTACLE_DISTANCE_M = 0.5   # stop if anything closer than this
CLEAR_DISTANCE_M    = 0.6   # resume only after this distance (hysteresis)
CHASSIS_MIN_RANGE_M = 0.15  # ignore robot's own body
FORWARD_CONE_DEG    = 30.0  # degrees either side of centre to watch
FORWARD_SPEED       = 0.2   # m/s


class MoveAndStop(Node):
    def __init__(self):
        super().__init__('move_and_stop_node')

        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.obstacle_ahead = False
        self.stopped = False
        self.last_nearest_forward = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
            callback_group=self.cb_group
        )

        self.timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.cb_group
        )

        self.get_logger().info(
            f'Move and stop node started — '
            f'will stop if obstacle within {OBSTACLE_DISTANCE_M}m'
        )

    def scan_callback(self, msg: LaserScan):
        cone_rad = math.radians(FORWARD_CONE_DEG)
        nearest_forward = None

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle) > cone_rad:
                continue
            if not math.isfinite(r):
                continue
            if r < max(msg.range_min, CHASSIS_MIN_RANGE_M) or r > msg.range_max:
                continue
            if nearest_forward is None or r < nearest_forward:
                nearest_forward = r

        with self._lock:
            prev = self.obstacle_ahead
            if nearest_forward is None:
                obstacle_now = False
            elif prev:
                obstacle_now = nearest_forward < CLEAR_DISTANCE_M
            else:
                obstacle_now = nearest_forward < OBSTACLE_DISTANCE_M
            self.obstacle_ahead = obstacle_now
            self.last_nearest_forward = nearest_forward

        if obstacle_now and not prev:
            self.get_logger().info(
                f'Obstacle within {OBSTACLE_DISTANCE_M}m — stopping!'
            )
        elif not obstacle_now and prev:
            self.get_logger().info('Path clear — moving forward')

        if nearest_forward is not None:
            self.get_logger().info(
                f'Nearest forward obstacle: {nearest_forward:.3f} m',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info(
                'No valid forward scan points',
                throttle_duration_sec=2.0
            )

    def control_loop(self):
        msg = Twist()

        with self._lock:
            obstacle = self.obstacle_ahead

        if obstacle:
            msg.linear.x = 0.0
            if not self.stopped:
                self.get_logger().info('Stopped.')
                self.stopped = True
        else:
            msg.linear.x = FORWARD_SPEED
            self.stopped = False

        self.cmd_pub.publish(msg)

    def destroy_node(self):
        self.cmd_pub.publish(Twist())  # safety stop
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveAndStop()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()