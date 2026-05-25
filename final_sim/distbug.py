#!/usr/bin/env python3
# PART 3 SIMULATION - SHRIRAM NARENDRAN

import math
import csv
import os
import json
from datetime import datetime
from typing import List, Tuple, Optional

import cv2

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from cv_bridge import CvBridge


class DistBugController(Node):
    def __init__(self):
        super().__init__('distbug_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/distbug_mode', 10)
        self.live_map_pub = self.create_publisher(String, '/distbug_live_map', 10)
        self.command_sub = self.create_subscription(
            String, '/distbug_command', self.command_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.declare_parameter('camera_topic', '/camera/image')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )

        self.timer = self.create_timer(0.05, self.control_loop)
        self.status_timer = self.create_timer(0.50, self.publish_mode_status)
        self.live_map_timer = self.create_timer(0.20, self.publish_live_map)

        # Hybrid pose estimate constant placeholders below
        self.yaw = None

        self.odom_x = None
        self.odom_y = None
        self.prev_odom_x = None
        self.prev_odom_y = None
        self.latest_distance_increment = 0.0

        self.est_x = None
        self.est_y = None

        # Scan data placeholder
        self.scan: Optional[LaserScan] = None

        # Camera data placeholder
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_stamp = None

        # Waypoints are NOT hardcoded anymore.
        # After mapping, the robot reads ~/ros2_ws/landmarks.csv and follows ONLY rows
        # where type == WAYPOINT. Each waypoint becomes (x, y, final_angle_deg).
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_goal_index = 0
        self.waypoints_loaded_from_landmarks = False
        self.waypoint_start_requested = False
        self.mapping_complete = False
        self.ui_status_text = "MAPPING"

        # NEW: After all waypoint travels are complete, return to origin/home at (0, 0).
        self.returning_to_origin_after_waypoints = False
        self.origin_goal_x = 0.0
        self.origin_goal_y = 0.0
        self.origin_final_yaw_deg = 0.0

        # If the UI asks to start waypoints during mapping, do not wait for the
        # full 360 scan. As soon as landmarks.csv contains at least one
        # WAYPOINT row, mapping is aborted and waypoint travel begins.
        self.asap_waypoint_check_period_s = 0.50
        self.asap_waypoint_last_check_time = -999.0

        # Initial states set
        self.state = 'CENTER_SCAN'
        self.finished = False

        # Memory of hit points, angle hit, minimum distance and for confirmation we have moved from this point
        self.hit_x = None
        self.hit_y = None
        self.hit_yaw = None
        self.mindist = None
        self.moved = False

        # Turning state, turning constants
        self.turn_target_yaw = None
        self.angular_speed = 0.6
        self.turn_correction = 3.0
        self.yaw_tolerance = math.radians(0.9)
        self.turn_start_time = None
        self.turn_timeout = None

        # final desired yaw at the end of a waypoint
        self.final_target_yaw = None

        # Part 3 centre scan + ray exploration settings
        self.center_scan_speed = 0.35
        self.center_scan_total = 2.0 * math.pi
        self.center_scan_step = math.radians(25)
        self.object_detect_range = 7.0
        self.object_detect_half_fov = math.radians(2.5)

        self.ray_object_detect_half_fov = math.radians(15.0)
        self.ray_photo_trigger_distance = 0.8

        self.object_standoff = 0.8
        self.ray_max_distance = 6.5
        self.ray_forward_speed = 0.25
        self.return_home_tolerance = 0.25

        self.center_scan_accumulated = 0.0
        self.center_scan_prev_yaw = None
        self.center_scan_started = False
        self.home_x = None
        self.home_y = None

        self.ray_yaw = None
        self.ray_target_x = None
        self.ray_target_y = None
        self.ray_distance = 0.0
        self.ray_photo_count = 0
        self.ray_last_photo_time = -999.0
        self.ray_photo_cooldown = 1.0
        self.ray_object_pending_photo = False
        self.ray_has_taken_photo = False

        # E-STOP feature.
        # Only active while the robot is deliberately approaching an object
        # or driving straight toward a waypoint. It watches the narrow 5 deg
        # front LiDAR cone and triggers if the reading suddenly drops below
        # 1 m from a previous safe reading.
        self.estop_triggered = False
        self.estop_previous_distance = None
        self.estop_half_fov = math.radians(2.5)
        self.estop_trigger_distance = 0.4
        self.estop_photo_saved = False
        self.estop_context = ""
        self.estop_photo_path = ""

        self.post_scan_delay_s = 10.0
        self.post_scan_delay_start = None

        # Distance keeping and dist bug constants (meters)
        self.STEP = 3.5
        self.SAFEDISTANCE = 0.45
        self.MOVEAWAY = 0.9
        self.WALL_FRONT_BLOCK = 0.60
        self.WALLDIST = 1.0
        self.GOALTOL = 1.1

        # Motion constants
        self.goal_forward_speed = 0.40
        self.goal_turn_gain = 1.8
        self.max_ang = 0.8
        self.wall_follow_speed = 0.10
        self.min_distance_increment = 0.0005

        # Map / logging settings
        self.map_x_min = -50.0
        self.map_x_max = 50.0
        self.map_y_min = -50.0
        self.map_y_max = 50.0

        self.output_dir = os.path.expanduser("~/ros2_ws")

        self.landmarks_csv_path = os.path.join(self.output_dir, "landmarks.csv")
        self.landmarks_csv_ready = False

        self.reset_landmarks_csv_on_startup()

        self.detected_waypoint_count = 0
        self.detected_object_count = 0
        self.detected_unknown_count = 0

        self.path_log_spacing = 0.01
        self.map_saved = False

        self.start_point: Optional[Tuple[float, float]] = None
        self.path_points: List[Tuple[float, float]] = []
        self.path_records: List[Tuple[float, float, float, str]] = []
        self.hit_points: List[Tuple[float, float]] = []
        self.leave_points: List[Tuple[float, float]] = []
        self.completed_waypoint_points: List[Tuple[float, float, float, int]] = []

        self.get_logger().info('DistBug controller started')

    # Utility functions
    def wrap_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quat_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_mode_status(self):
        msg = String()
        msg.data = self.get_mode_status_text()
        self.mode_pub.publish(msg)

    def publish_live_map(self):
        """
        Publish lightweight live-map data for the Tkinter mission-control UI.

        Topic:
          /distbug_live_map

        Type:
          std_msgs/String containing JSON

        This does not change robot behaviour. It only mirrors the controller's
        current path/pose/landmark state so the UI can draw a live map.
        """
        if self.est_x is None or self.est_y is None or self.yaw is None:
            return

        # Keep the packet bounded so very long missions do not slow the UI.
        max_path_points = 2500
        path_to_send = self.path_points[-max_path_points:]

        if self.returning_to_origin_after_waypoints:
            current_goal = {
                "x": self.origin_goal_x,
                "y": self.origin_goal_y,
                "label": "HOME",
            }
        elif self.waypoints and self.current_goal_index < len(self.waypoints):
            gx, gy, _ = self.waypoints[self.current_goal_index]
            current_goal = {
                "x": gx,
                "y": gy,
                "label": f"WP{self.current_goal_index + 1}",
            }
        else:
            current_goal = None

        payload = {
            "state": self.state,
            "status": self.get_mode_status_text(),
            "finished": self.finished,
            "pose": {
                "x": self.est_x,
                "y": self.est_y,
                "yaw": self.yaw,
            },
            "start": {
                "x": self.start_point[0],
                "y": self.start_point[1],
            } if self.start_point is not None else None,
            "home": {
                "x": self.home_x,
                "y": self.home_y,
            } if self.home_x is not None and self.home_y is not None else None,
            "origin": {
                "x": self.origin_goal_x,
                "y": self.origin_goal_y,
            },
            "current_goal": current_goal,
            "path": [
                {"x": x, "y": y}
                for x, y in path_to_send
            ],
            "hits": [
                {"x": x, "y": y}
                for x, y in self.hit_points
            ],
            "leaves": [
                {"x": x, "y": y}
                for x, y in self.leave_points
            ],
            "waypoints": [
                {"x": x, "y": y, "yaw_deg": yaw_deg, "index": i + 1}
                for i, (x, y, yaw_deg) in enumerate(self.waypoints)
            ],
            "completed": [
                {"x": x, "y": y, "yaw": yaw, "index": idx}
                for x, y, yaw, idx in self.completed_waypoint_points
            ],
            "ray": {
                "active": self.ray_yaw is not None,
                "yaw": self.ray_yaw,
                "target_x": self.ray_target_x,
                "target_y": self.ray_target_y,
                "distance": self.ray_distance,
                "max_distance": self.ray_max_distance,
            },
            "counts": {
                "waypoints": self.detected_waypoint_count,
                "objects": self.detected_object_count,
                "unknown": self.detected_unknown_count,
                "photos": self.ray_photo_count,
            },
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.live_map_pub.publish(msg)

    def get_mode_status_text(self) -> str:
        if self.estop_triggered or self.state == 'ESTOP':
            return "ESTOP TRIGGERED"

        if self.finished:
            return "FINISHED"

        mapping_states = {
            'CENTER_SCAN', 'RAY_APPROACH_OBJECT', 'RAY_CONTINUE',
            'RAY_INIT_TURN_AFTER_OBJECT', 'RAY_BOUNDARY_FOLLOW',
            'RETURN_GO_HOME', 'RETURN_INIT_TURN_AFTER_HIT',
            'RETURN_BOUNDARY_FOLLOW', 'HOME_ALIGN_TO_RAY',
            'CENTER_ROTATE_5', 'POST_SCAN_DELAY',
        }

        if self.state in mapping_states:
            if self.waypoint_start_requested:
                return f"MAPPING / waypoint start queued / state={self.state}"
            return f"MAPPING / state={self.state}"

        if self.state == 'WAIT_FOR_WAYPOINT_COMMAND':
            return f"MAPPING DONE / waiting for Go To Waypoints / loaded={len(self.waypoints)}"

        if self.state in {'GO_TO_GOAL', 'INIT_TURN_AFTER_HIT', 'BOUNDARY_FOLLOW', 'FINAL_ALIGN', 'FINAL_ALIGN_ORIGIN'}:
            if self.returning_to_origin_after_waypoints:
                return f"RETURNING HOME TO ORIGIN (0,0) / state={self.state}"
            return f"WAYPOINT TRAVEL / {self.current_goal_index + 1}/{len(self.waypoints)} / state={self.state}"

        return f"UNKNOWN / state={self.state}"

    def command_callback(self, msg: String):
        command = msg.data.strip().upper()

        if self.estop_triggered:
            self.publish_stop()
            self.get_logger().error('ESTOP TRIGGERED. Robot is locked stopped. Restart the node to clear E-STOP.')
            return

        if command == 'START_WAYPOINTS':
            self.waypoint_start_requested = True
            self.get_logger().info('UI command received: START_WAYPOINTS')

            if self.try_start_waypoint_travel_if_available():
                return

            self.get_logger().info(
                f'Waypoint start queued. Current state is {self.state}; '
                'robot will start as soon as landmarks.csv contains a WAYPOINT row.'
            )
            return

        if command == 'STOP':
            self.waypoint_start_requested = False
            self.publish_stop()
            if self.state in {'GO_TO_GOAL', 'INIT_TURN_AFTER_HIT', 'BOUNDARY_FOLLOW', 'FINAL_ALIGN', 'FINAL_ALIGN_ORIGIN'}:
                self.state = 'WAIT_FOR_WAYPOINT_COMMAND'
                self.returning_to_origin_after_waypoints = False
            self.get_logger().warn('UI command received: STOP. Robot stopped and waypoint travel paused.')
            return

        self.get_logger().warn(f'Unknown UI command: {msg.data}')

    def landmarks_csv_has_waypoint(self) -> bool:
        if not os.path.exists(self.landmarks_csv_path):
            return False

        try:
            if os.path.getsize(self.landmarks_csv_path) <= 0:
                return False

            with open(self.landmarks_csv_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if (row.get('type') or '').strip().upper() == 'WAYPOINT':
                        return True
        except Exception as e:
            self.get_logger().warn(f'Could not check landmarks CSV yet: {e}')

        return False

    def try_start_waypoint_travel_if_available(self) -> bool:
        waypoint_states = {'GO_TO_GOAL', 'INIT_TURN_AFTER_HIT', 'BOUNDARY_FOLLOW', 'FINAL_ALIGN', 'FINAL_ALIGN_ORIGIN'}
        if self.state in waypoint_states:
            return True

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.asap_waypoint_last_check_time < self.asap_waypoint_check_period_s:
            return False
        self.asap_waypoint_last_check_time = now

        if not self.landmarks_csv_has_waypoint():
            return False

        self.get_logger().info(
            'WAYPOINT row found in landmarks.csv. Aborting remaining mapping and starting waypoint travel now.'
        )
        return self.start_waypoint_travel_from_landmarks(keep_request_on_fail=True)

    def load_waypoints_from_landmarks_csv(self) -> bool:
        self.ensure_landmarks_csv()

        if not os.path.exists(self.landmarks_csv_path):
            self.get_logger().warn(f'Cannot load waypoints. CSV does not exist: {self.landmarks_csv_path}')
            return False

        loaded: List[Tuple[str, float, float, float]] = []

        try:
            with open(self.landmarks_csv_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    landmark_type = (row.get('type') or '').strip().upper()
                    if landmark_type != 'WAYPOINT':
                        continue

                    label = (row.get('label') or f'waypoint_{len(loaded) + 1}').strip()
                    try:
                        x = float(row.get('x', ''))
                        y = float(row.get('y', ''))
                    except ValueError:
                        self.get_logger().warn(f'Skipping waypoint row with invalid coordinates: {row}')
                        continue

                    loaded.append((label, x, y, 0.0))
        except Exception as e:
            self.get_logger().warn(f'Failed to read landmarks CSV: {e}')
            return False

        if not loaded:
            self.waypoints = []
            self.get_logger().warn('No WAYPOINT rows found in landmarks.csv. Robot will not start waypoint travel.')
            return False

        def waypoint_sort_key(item):
            label = item[0].lower()
            digits = ''.join(ch for ch in label if ch.isdigit())
            return int(digits) if digits else 999999

        loaded.sort(key=waypoint_sort_key)
        self.waypoints = [(x, y, yaw_deg) for _, x, y, yaw_deg in loaded]
        self.current_goal_index = 0
        self.waypoints_loaded_from_landmarks = True
        self.returning_to_origin_after_waypoints = False

        pretty = ', '.join(
            f'{label}=({x:.2f},{y:.2f})' for label, x, y, _ in loaded
        )
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint(s) from landmarks.csv: {pretty}')
        return True

    def start_waypoint_travel_from_landmarks(self, keep_request_on_fail: bool = False) -> bool:
        self.publish_stop()
        self.reset_distbug_memory()

        if not self.load_waypoints_from_landmarks_csv():
            self.state = 'WAIT_FOR_WAYPOINT_COMMAND'
            if not keep_request_on_fail:
                self.waypoint_start_requested = False
            return False

        self.state = 'GO_TO_GOAL'
        self.waypoint_start_requested = False
        self.mapping_complete = True
        self.finished = False
        self.returning_to_origin_after_waypoints = False
        self.get_logger().info('Starting waypoint travel using ONLY WAYPOINT rows from landmarks.csv.')
        return True

    def start_return_to_origin_after_waypoints(self):
        """
        After all real WAYPOINT rows have been completed, drive back to origin (0,0)
        using the existing DistBug GO_TO_GOAL / BOUNDARY_FOLLOW logic.
        """
        self.publish_stop()
        self.reset_distbug_memory()

        self.returning_to_origin_after_waypoints = True
        self.final_target_yaw = None
        self.turn_target_yaw = None
        self.turn_start_time = None
        self.turn_timeout = None

        self.state = 'GO_TO_GOAL'

        self.get_logger().info(
            f'All waypoints completed. Now returning home to origin '
            f'({self.origin_goal_x:.2f}, {self.origin_goal_y:.2f}).'
        )

    def check_point(self, x: float, y: float, goalx: float, goaly: float) -> bool:
        dx = goalx - x
        dy = goaly - y
        distance = math.hypot(dx, dy)
        return distance <= self.GOALTOL

    def check_pointv2(self, x: float, y: float, goalx: float, goaly: float) -> bool:
        dx = goalx - x
        dy = goaly - y
        distance = math.hypot(dx, dy)
        return distance <= 0.3

    def seen_before(self, x: float, y: float) -> bool:
        for hx, hy in self.hit_points:
            if self.check_pointv2(x, y, hx, hy):
                return True
        return False

    def get_relative_goal_location(self, goalx: float, goaly: float) -> Tuple[float, float]:
        dx = goalx - self.est_x
        dy = goaly - self.est_y
        distance = math.hypot(dx, dy)

        goalB = math.atan2(dy, dx)
        rot = self.wrap_angle(goalB - self.yaw)

        return distance, rot

    def reset_distbug_memory(self):
        self.hit_x = None
        self.hit_y = None
        self.hit_yaw = None
        self.mindist = None
        self.moved = False
        self.reset_estop_tracker()

    def reset_estop_tracker(self):
        self.estop_previous_distance = None

    def save_estop_photo(self, context: str, current_distance: float, previous_distance: Optional[float]):
        os.makedirs(self.output_dir, exist_ok=True)

        if self.latest_image is None:
            self.get_logger().warn('ESTOP TRIGGERED, but no camera image available to save')
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        safe_context = ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in context.lower())
        image_path = os.path.join(
            self.output_dir,
            f"ESTOP_TRIGGERED_{safe_context}_{timestamp}.png"
        )

        success = cv2.imwrite(image_path, self.latest_image)
        if success:
            self.estop_photo_saved = True
            self.estop_photo_path = image_path
            self.get_logger().error(
                f'ESTOP TRIGGERED photo saved: {image_path} '
                f'context={context} previous_5deg={previous_distance} current_5deg={current_distance:.3f}m'
            )
        else:
            self.get_logger().warn('ESTOP TRIGGERED, but failed to save camera image')

    def trigger_estop(self, context: str, current_distance: float, previous_distance: Optional[float]):
        self.publish_stop()
        self.estop_triggered = True
        self.estop_context = context
        self.state = 'ESTOP'
        self.save_estop_photo(context, current_distance, previous_distance)

        msg = String()
        msg.data = "ESTOP TRIGGERED"
        self.mode_pub.publish(msg)

        self.get_logger().error(
            f'ESTOP TRIGGERED: context={context} '
            f'previous_5deg={previous_distance} current_5deg={current_distance:.3f}m '
            f'pose=({self.est_x:.3f},{self.est_y:.3f}) yaw={math.degrees(self.yaw):.2f}deg'
        )

    def check_estop_during_approach(self, context: str, current_distance: float) -> bool:
        previous_distance = self.estop_previous_distance

        if previous_distance is not None:
            if previous_distance >= self.estop_trigger_distance and current_distance < self.estop_trigger_distance:
                self.trigger_estop(context, current_distance, previous_distance)
                return True

        self.estop_previous_distance = current_distance
        return False

    def record_path_point(self):
        if self.est_x is None or self.est_y is None or self.yaw is None:
            return

        if self.start_point is None:
            self.start_point = (self.est_x, self.est_y)

        if not self.path_points:
            self.path_points.append((self.est_x, self.est_y))
            self.path_records.append((self.est_x, self.est_y, self.yaw, self.state))
            return

        last_x, last_y = self.path_points[-1]
        if math.hypot(self.est_x - last_x, self.est_y - last_y) >= self.path_log_spacing:
            self.path_points.append((self.est_x, self.est_y))
            self.path_records.append((self.est_x, self.est_y, self.yaw, self.state))

    def save_waypoint_photo(self, waypoint_index: int):
        os.makedirs(self.output_dir, exist_ok=True)

        if self.latest_image is None:
            self.get_logger().warn(
                f'No camera image available to save for waypoint {waypoint_index}'
            )
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(
            self.output_dir,
            f"waypoint_{waypoint_index}_photo_{timestamp}.png"
        )

        success = cv2.imwrite(image_path, self.latest_image)

        if success:
            self.get_logger().info(
                f'Saved waypoint {waypoint_index} photo: {image_path}'
            )
        else:
            self.get_logger().warn(
                f'Failed to save waypoint {waypoint_index} photo'
            )

    def write_landmarks_csv_header(self):
        os.makedirs(self.output_dir, exist_ok=True)

        with open(self.landmarks_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "label",
                "type",
                "colour",
                "x",
                "y",
                "robot_x",
                "robot_y",
                "robot_yaw_rad",
                "relative_distance_m",
                "ray_progress_m",
                "image_path",
                "timestamp",
            ])

    def reset_landmarks_csv_on_startup(self):
        self.write_landmarks_csv_header()
        self.landmarks_csv_ready = True
        self.get_logger().info(f"Cleared landmarks CSV for new run: {self.landmarks_csv_path}")

    def ensure_landmarks_csv(self):
        if self.landmarks_csv_ready:
            return

        self.write_landmarks_csv_header()
        self.landmarks_csv_ready = True
        self.get_logger().info(f"Landmarks CSV ready: {self.landmarks_csv_path}")

    def classify_latest_image_hsv(self) -> Tuple[str, str]:
        if self.latest_image is None:
            return "UNKNOWN", "no_image"

        img = self.latest_image
        h, w = img.shape[:2]

        x1 = int(w * 0.25)
        x2 = int(w * 0.75)
        y1 = int(h * 0.20)
        y2 = int(h * 0.80)
        crop = img[y1:y2, x1:x2]

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        red1 = cv2.inRange(hsv, (0, 80, 70), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 80, 70), (179, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        orange = cv2.inRange(hsv, (11, 80, 70), (24, 255, 255))
        yellow = cv2.inRange(hsv, (25, 70, 70), (40, 255, 255))
        blue = cv2.inRange(hsv, (90, 70, 60), (135, 255, 255))

        total_pixels = max(1, crop.shape[0] * crop.shape[1])
        red_ratio = cv2.countNonZero(red) / total_pixels
        orange_ratio = cv2.countNonZero(orange) / total_pixels
        yellow_ratio = cv2.countNonZero(yellow) / total_pixels
        blue_ratio = cv2.countNonZero(blue) / total_pixels
        warm_ratio = red_ratio + orange_ratio + yellow_ratio

        min_colour_ratio = 0.01

        self.get_logger().info(
            f'HSV ratios: blue={blue_ratio:.3f}, red={red_ratio:.3f}, '
            f'orange={orange_ratio:.3f}, yellow={yellow_ratio:.3f}'
        )

        if blue_ratio >= min_colour_ratio and blue_ratio > warm_ratio:
            return "WAYPOINT", "blue"

        if warm_ratio >= min_colour_ratio:
            if red_ratio >= orange_ratio and red_ratio >= yellow_ratio:
                return "OBJECT", "red"
            if orange_ratio >= red_ratio and orange_ratio >= yellow_ratio:
                return "OBJECT", "orange"
            return "OBJECT", "yellow"

        return "UNKNOWN", "unknown"

    def estimate_landmark_coordinate(self, relative_distance: float) -> Tuple[float, float]:
        lx = self.est_x + relative_distance * math.cos(self.yaw)
        ly = self.est_y + relative_distance * math.sin(self.yaw)
        return lx, ly

    def append_landmark_csv(
        self,
        label: str,
        landmark_type: str,
        colour_name: str,
        landmark_x: float,
        landmark_y: float,
        relative_distance: float,
        image_path: str,
        timestamp: str,
    ):
        self.ensure_landmarks_csv()

        with open(self.landmarks_csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                label,
                landmark_type,
                colour_name,
                f"{landmark_x:.4f}",
                f"{landmark_y:.4f}",
                f"{self.est_x:.4f}",
                f"{self.est_y:.4f}",
                f"{self.yaw:.6f}",
                f"{relative_distance:.4f}",
                f"{self.ray_distance:.4f}",
                image_path,
                timestamp,
            ])

    def save_ray_object_photo(self, object_distance: float):
        os.makedirs(self.output_dir, exist_ok=True)

        if self.latest_image is None:
            self.get_logger().warn('Object reached, but no camera image available to save')
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.ray_last_photo_time < self.ray_photo_cooldown:
            return

        self.ray_last_photo_time = now
        landmark_type, colour_name = self.classify_latest_image_hsv()

        if landmark_type == "WAYPOINT":
            self.detected_waypoint_count += 1
            label = f"waypoint_{self.detected_waypoint_count}"
        elif landmark_type == "OBJECT":
            self.detected_object_count += 1
            label = f"object_{self.detected_object_count}"
        else:
            self.detected_unknown_count += 1
            label = f"unknown_{self.detected_unknown_count}"

        self.ray_photo_count += 1

        landmark_x, landmark_y = self.estimate_landmark_coordinate(object_distance)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        image_path = os.path.join(
            self.output_dir,
            f"{label}_{colour_name}_{timestamp}.png"
        )

        success = cv2.imwrite(image_path, self.latest_image)

        if success:
            self.append_landmark_csv(
                label,
                landmark_type,
                colour_name,
                landmark_x,
                landmark_y,
                object_distance,
                image_path,
                timestamp,
            )

            self.get_logger().info(
                f'Saved {landmark_type} photo: {image_path} '
                f'label={label} colour={colour_name} '
                f'landmark=({landmark_x:.2f},{landmark_y:.2f}) '
                f'object_distance={object_distance:.2f}m '
                f'ray_progress={self.ray_distance:.2f}m '
                f'yaw={math.degrees(self.yaw):.1f}deg'
            )
        else:
            self.get_logger().warn(f'Failed to save landmark photo for {label}')

    def update_ray_progress_from_pose(self):
        if self.home_x is None or self.home_y is None or self.ray_yaw is None:
            self.ray_distance = 0.0
            return

        dx = self.est_x - self.home_x
        dy = self.est_y - self.home_y
        projection = dx * math.cos(self.ray_yaw) + dy * math.sin(self.ray_yaw)
        self.ray_distance = max(0.0, projection)

    def start_new_ray(self):
        self.publish_stop()
        self.reset_estop_tracker()
        self.ray_yaw = self.yaw
        self.ray_target_x = self.home_x + self.ray_max_distance * math.cos(self.ray_yaw)
        self.ray_target_y = self.home_y + self.ray_max_distance * math.sin(self.ray_yaw)
        self.ray_distance = 0.0
        self.ray_object_pending_photo = True
        self.ray_has_taken_photo = False
        self.reset_distbug_memory()
        self.state = 'RAY_APPROACH_OBJECT'
        self.get_logger().info(
            f'Object detected from centre within {self.object_detect_range:.1f}m. '
            f'Committing to ray yaw={math.degrees(self.ray_yaw):.1f}deg, '
            f'target=({self.ray_target_x:.2f},{self.ray_target_y:.2f}).'
        )

    def start_return_home(self):
        self.publish_stop()
        self.reset_distbug_memory()
        self.state = 'RETURN_GO_HOME'
        self.get_logger().info(
            f'Ray progress {self.ray_distance:.2f}m reached/over {self.ray_max_distance:.2f}m. '
            'Turning back and returning to centre with obstacle avoidance.'
        )

    def start_ray_object_avoidance(self):
        distance_to_ray_end = math.hypot(self.ray_target_x - self.est_x, self.ray_target_y - self.est_y)
        self.hit_x = self.est_x
        self.hit_y = self.est_y
        self.hit_yaw = self.yaw
        self.mindist = distance_to_ray_end
        self.moved = False

        if not self.seen_before(self.est_x, self.est_y):
            self.hit_points.append((self.est_x, self.est_y))
            self.get_logger().info(
                f'RAY ADDED HIT POINT at ({self.est_x:.3f}, {self.est_y:.3f})'
            )

        self.publish_stop()
        self.start_turn_relative(-math.pi / 2.5)
        self.state = 'RAY_INIT_TURN_AFTER_OBJECT'

    def drive_along_ray_step(self, speed: float):
        yaw_error = self.wrap_angle(self.ray_yaw - self.yaw)
        twist = Twist()

        if abs(yaw_error) > math.radians(8.0):
            twist.linear.x = 0.04
            twist.angular.z = max(-self.max_ang, min(self.max_ang, self.goal_turn_gain * yaw_error))
        else:
            twist.linear.x = speed
            twist.angular.z = max(-0.35, min(0.35, self.goal_turn_gain * yaw_error))

        self.cmd_pub.publish(twist)

    def save_results(self):
        if self.map_saved:
            return

        self.map_saved = True
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        full_png = os.path.join(self.output_dir, f"distbug_map_FULL_{timestamp}.png")
        zoom_png = os.path.join(self.output_dir, f"distbug_map_ZOOM_{timestamp}.png")
        csv_path = os.path.join(self.output_dir, f"distbug_path_{timestamp}.csv")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw_rad", "state"])
            for row in self.path_records:
                writer.writerow(row)

        def draw_map(ax):
            if self.path_points:
                xs = [p[0] for p in self.path_points]
                ys = [p[1] for p in self.path_points]
                ax.plot(xs, ys, linewidth=2.5, label="Driven path")

            if self.start_point is not None:
                ax.scatter(*self.start_point, s=100, marker='o', label="Start")
                ax.text(*self.start_point, "  Start")

            for i, (wx, wy, _) in enumerate(self.waypoints, start=1):
                ax.scatter(wx, wy, s=100, marker='s')
                ax.text(wx, wy + 0.2, f"  WP{i}")

            # Draw final return-home/origin point.
            ax.scatter(self.origin_goal_x, self.origin_goal_y, s=120, marker='*', label="Origin/Home")
            ax.text(self.origin_goal_x, self.origin_goal_y + 0.2, "  Home (0,0)")

            if self.hit_points:
                hx = [p[0] for p in self.hit_points]
                hy = [p[1] for p in self.hit_points]
                ax.scatter(hx, hy, s=80, marker='x', label="Hits")
                for i, (x, y) in enumerate(self.hit_points, start=1):
                    ax.text(x, y, f"  H{i}")

            if self.leave_points:
                lx = [p[0] for p in self.leave_points]
                ly = [p[1] for p in self.leave_points]
                ax.scatter(lx, ly, s=80, marker='^', label="Leaves")
                for i, (x, y) in enumerate(self.leave_points, start=1):
                    ax.text(x, y, f"  L{i}")

            for x, y, yaw, idx in self.completed_waypoint_points:
                dx = 0.3 * math.cos(yaw)
                dy = 0.3 * math.sin(yaw)
                ax.arrow(
                    x, y, dx, dy,
                    head_width=0.1,
                    head_length=0.1,
                    length_includes_head=True
                )

            ax.grid(True)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_aspect('equal')

        fig1, ax1 = plt.subplots(figsize=(10, 10))
        draw_map(ax1)

        ax1.set_xlim(self.map_x_min, self.map_x_max)
        ax1.set_ylim(self.map_y_min, self.map_y_max)
        ax1.set_title("PIONEER VOYAGE (Full World)")
        ax1.legend()

        plt.tight_layout()
        plt.savefig(full_png, dpi=350)
        plt.close(fig1)

        fig2, ax2 = plt.subplots(figsize=(10, 10))
        draw_map(ax2)

        xs, ys = [], []

        if self.path_points:
            xs += [p[0] for p in self.path_points]
            ys += [p[1] for p in self.path_points]

        if self.start_point:
            xs.append(self.start_point[0])
            ys.append(self.start_point[1])

        xs.append(self.origin_goal_x)
        ys.append(self.origin_goal_y)

        for (wx, wy, _) in self.waypoints:
            xs.append(wx)
            ys.append(wy)

        for p in self.hit_points:
            xs.append(p[0])
            ys.append(p[1])

        for p in self.leave_points:
            xs.append(p[0])
            ys.append(p[1])

        for x, y, _, _ in self.completed_waypoint_points:
            xs.append(x)
            ys.append(y)

        if xs and ys:
            padding = 0.5
            ax2.set_xlim(min(xs) - padding, max(xs) + padding)
            ax2.set_ylim(min(ys) - padding, max(ys) + padding)

        ax2.set_title("PIONEER VOYAGE (Zoomed View)")
        ax2.legend()

        plt.tight_layout()
        plt.savefig(zoom_png, dpi=400)
        plt.close(fig2)

        self.get_logger().info(f"Saved FULL map: {full_png}")
        self.get_logger().info(f"Saved ZOOM map: {zoom_png}")
        self.get_logger().info(f"Saved CSV: {csv_path}")

    # Sensor callbacks
    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        if self.prev_odom_x is None or self.prev_odom_y is None:
            self.prev_odom_x = self.odom_x
            self.prev_odom_y = self.odom_y

            if self.est_x is None or self.est_y is None:
                self.est_x = self.odom_x
                self.est_y = self.odom_y
                self.start_point = (self.est_x, self.est_y)
            return

        dx = self.odom_x - self.prev_odom_x
        dy = self.odom_y - self.prev_odom_y
        dist_inc = math.hypot(dx, dy)
        self.latest_distance_increment += dist_inc

        self.prev_odom_x = self.odom_x
        self.prev_odom_y = self.odom_y

    def imu_callback(self, msg: Imu):
        self.yaw = self.quat_to_yaw(msg.orientation)

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')

    # Pose estimation
    def consume_forward_increment(self):
        dist_inc = self.latest_distance_increment
        if dist_inc > self.min_distance_increment:
            self.est_x += dist_inc * math.cos(self.yaw)
            self.est_y += dist_inc * math.sin(self.yaw)
        self.latest_distance_increment = 0.0
        return dist_inc

    # LaserScan helpers
    def get_range_at_bearing(self, bearing_rad: float) -> float:
        if self.scan is None:
            return 0.0

        if bearing_rad < self.scan.angle_min or bearing_rad > self.scan.angle_max:
            return self.scan.range_max

        idx = int((bearing_rad - self.scan.angle_min) / self.scan.angle_increment)
        idx = max(0, min(idx, len(self.scan.ranges) - 1))

        r = self.scan.ranges[idx]
        if math.isinf(r) or math.isnan(r):
            return self.scan.range_max
        return r

    def min_range_in_window(self, center_rad: float, half_width_rad: float) -> float:
        if self.scan is None:
            return 0.0

        rmin = self.scan.range_max
        angle = center_rad - half_width_rad
        end = center_rad + half_width_rad

        while angle <= end:
            r = self.get_range_at_bearing(angle)
            rmin = min(rmin, r)
            angle += self.scan.angle_increment * 3.0

        return rmin

    def front_distance(self) -> float:
        return self.min_range_in_window(0.0, math.radians(60.0))

    # Turn helpers
    def start_turn_relative(self, delta_rad: float):
        self.turn_target_yaw = self.wrap_angle(self.yaw + delta_rad)
        self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

        yaw_error = self.wrap_angle(self.turn_target_yaw - self.yaw)
        ideal_time = abs(yaw_error) / self.angular_speed if self.angular_speed > 0.0 else 0.0
        self.turn_timeout = ideal_time * self.turn_correction

    def start_turn_absolute(self, target_yaw_rad: float):
        self.turn_target_yaw = self.wrap_angle(target_yaw_rad)
        self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

        yaw_error = self.wrap_angle(self.turn_target_yaw - self.yaw)
        ideal_time = abs(yaw_error) / self.angular_speed if self.angular_speed > 0.0 else 0.0
        self.turn_timeout = ideal_time * self.turn_correction

    def perform_turn_step(self) -> bool:
        yaw_error = self.wrap_angle(self.turn_target_yaw - self.yaw)
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.turn_start_time

        if abs(yaw_error) < self.yaw_tolerance:
            self.publish_stop()
            return True

        if elapsed > self.turn_timeout:
            self.publish_stop()
            self.get_logger().warn('Turn stopped by timeout safety')
            return True

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed if yaw_error > 0.0 else -self.angular_speed
        self.cmd_pub.publish(twist)
        return False

    # Part 3 centre scan + ray exploration logic
    def center_scan_step_method(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if not self.center_scan_started:
            self.center_scan_started = True
            self.center_scan_prev_yaw = self.yaw
            self.center_scan_accumulated = 0.0
            self.home_x = self.est_x
            self.home_y = self.est_y
            self.publish_stop()
            self.get_logger().info(
                'Starting Part 3 centre scan. Rotating ACW at centre. '
                'If object appears in 5 deg front cone within 7 m, robot will approach, photo, avoid, '
                'continue to 6.5 m ray distance, return home, rotate +5 deg, and repeat.'
            )
            return

        yaw_delta = self.wrap_angle(self.yaw - self.center_scan_prev_yaw)
        self.center_scan_prev_yaw = self.yaw
        self.center_scan_accumulated += abs(yaw_delta)

        if self.center_scan_accumulated >= self.center_scan_total:
            self.publish_stop()
            self.post_scan_delay_start = now
            self.state = 'POST_SCAN_DELAY'
            self.get_logger().info(
                f'Centre 360 scan complete. Landmark photos saved: {self.ray_photo_count}. '
                f'Waiting {self.post_scan_delay_s:.1f}s before normal waypoint following.'
            )
            return

        object_distance = self.min_range_in_window(0.0, self.object_detect_half_fov)
        if object_distance <= self.object_detect_range:
            self.start_new_ray()
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.center_scan_speed
        self.cmd_pub.publish(twist)

    def ray_approach_object_step(self):
        self.consume_forward_increment()
        self.update_ray_progress_from_pose()

        if self.ray_distance >= self.ray_max_distance:
            self.start_return_home()
            return

        object_distance = self.min_range_in_window(0.0, self.ray_object_detect_half_fov)
        estop_distance = self.min_range_in_window(0.0, self.estop_half_fov)
        front = self.front_distance()

        self.get_logger().info(
            f'RAY_APPROACH: progress={self.ray_distance:.2f}/{self.ray_max_distance:.2f}m '
            f'object30deg={object_distance:.2f}m estop5deg={estop_distance:.2f}m front={front:.2f}m'
        )

        if self.check_estop_during_approach('OBJECT_APPROACH', estop_distance):
            return

        if object_distance <= self.ray_photo_trigger_distance or front <= self.object_standoff:
            self.publish_stop()
            photo_distance = min(object_distance, front)
            self.save_ray_object_photo(photo_distance)
            self.ray_object_pending_photo = False
            self.ray_has_taken_photo = True

            if self.waypoint_start_requested and self.try_start_waypoint_travel_if_available():
                return

            if self.ray_distance >= self.ray_max_distance:
                self.start_return_home()
            else:
                self.start_ray_object_avoidance()
            return

        if object_distance <= self.object_detect_range:
            self.drive_along_ray_step(self.ray_forward_speed)
        else:
            self.state = 'RAY_CONTINUE'
            self.get_logger().info(
                'Object no longer in 30 deg ray cone while approaching. Continuing along ray.'
            )

    def ray_continue_step(self):
        self.reset_estop_tracker()
        self.consume_forward_increment()
        self.update_ray_progress_from_pose()

        if self.ray_distance >= self.ray_max_distance:
            self.start_return_home()
            return

        object_distance = self.min_range_in_window(0.0, self.ray_object_detect_half_fov)
        front = self.front_distance()

        self.get_logger().info(
            f'RAY_CONTINUE: progress={self.ray_distance:.2f}/{self.ray_max_distance:.2f}m '
            f'object30deg={object_distance:.2f}m front={front:.2f}m'
        )

        if self.ray_distance < self.ray_max_distance and object_distance <= self.object_detect_range:
            self.ray_object_pending_photo = True
            self.state = 'RAY_APPROACH_OBJECT'
            return

        if front <= self.ray_photo_trigger_distance:
            self.publish_stop()
            self.save_ray_object_photo(front)
            self.ray_object_pending_photo = False
            self.ray_has_taken_photo = True

            if self.waypoint_start_requested and self.try_start_waypoint_travel_if_available():
                return

            self.start_ray_object_avoidance()
            return

        if front < self.SAFEDISTANCE:
            self.start_ray_object_avoidance()
            return

        self.drive_along_ray_step(self.ray_forward_speed)

    def ray_init_turn_after_object_step(self):
        done = self.perform_turn_step()
        if done:
            self.state = 'RAY_BOUNDARY_FOLLOW'
            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.get_logger().info('Completed ray obstacle turn. Entering RAY_BOUNDARY_FOLLOW.')

    def ray_boundary_follow_step(self):
        self.consume_forward_increment()
        self.update_ray_progress_from_pose()

        if self.ray_distance >= self.ray_max_distance:
            self.start_return_home()
            return

        distance = math.hypot(self.ray_target_x - self.est_x, self.ray_target_y - self.est_y)
        angle = self.wrap_angle(math.atan2(self.ray_target_y - self.est_y, self.ray_target_x - self.est_x) - self.yaw)

        sqr = math.hypot(self.hit_x - self.est_x, self.hit_y - self.est_y)
        if sqr > self.MOVEAWAY:
            self.moved = True

        if distance < self.mindist:
            self.mindist = distance

        freespacetogoal = self.get_range_at_bearing(angle)
        front = self.front_distance()
        left = self.get_range_at_bearing(1.571)

        self.get_logger().info(
            f'RAY_BOUNDARY: progress={self.ray_distance:.2f}m d_end={distance:.2f} '
            f'F={freespacetogoal:.2f} min={self.mindist:.2f} left={left:.2f} front={front:.2f}'
        )

        if self.moved and distance - freespacetogoal <= self.mindist - self.STEP:
            self.publish_stop()
            self.leave_points.append((self.est_x, self.est_y))
            self.state = 'RAY_CONTINUE'
            self.get_logger().info('RAY LEAVE boundary and continue along committed ray.')
            return

        twist = Twist()

        if front < self.WALL_FRONT_BLOCK:
            twist.linear.x = 0.0
            twist.angular.z = -self.max_ang
            self.cmd_pub.publish(twist)
            return

        err = left - self.WALLDIST

        if err > 0.20:
            twist.linear.x = 0.10
            twist.angular.z = 0.6
        elif err > 0.01:
            twist.linear.x = 0.10
            twist.angular.z = 0.15
        elif err < -0.05:
            twist.linear.x = 0.10
            twist.angular.z = -0.6
        elif err < -0.005:
            twist.linear.x = 0.10
            twist.angular.z = -0.15
        else:
            twist.linear.x = 0.10
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def return_go_home_step(self):
        self.consume_forward_increment()

        distance = math.hypot(self.home_x - self.est_x, self.home_y - self.est_y)
        angle = self.wrap_angle(math.atan2(self.home_y - self.est_y, self.home_x - self.est_x) - self.yaw)
        freespacetogoal = self.get_range_at_bearing(angle)
        front = self.front_distance()

        self.get_logger().info(
            f'RETURN_HOME: d_home={distance:.2f} bearing={math.degrees(angle):.1f}deg '
            f'front={front:.2f} F={freespacetogoal:.2f}'
        )

        if distance <= self.return_home_tolerance:
            self.publish_stop()
            self.est_x = self.home_x
            self.est_y = self.home_y
            self.start_turn_absolute(self.ray_yaw)
            self.state = 'HOME_ALIGN_TO_RAY'
            self.get_logger().info('Returned to centre. Re-aligning to previous ray yaw.')
            return

        if front < self.SAFEDISTANCE or freespacetogoal < self.SAFEDISTANCE:
            self.hit_x = self.est_x
            self.hit_y = self.est_y
            self.hit_yaw = self.yaw
            self.mindist = distance
            self.moved = False

            if not self.seen_before(self.est_x, self.est_y):
                self.hit_points.append((self.est_x, self.est_y))
                self.get_logger().info(
                    f'RETURN ADDED HIT POINT at ({self.est_x:.3f}, {self.est_y:.3f})'
                )

            self.publish_stop()
            self.start_turn_relative(-math.pi / 2.5)
            self.state = 'RETURN_INIT_TURN_AFTER_HIT'
            return

        twist = Twist()
        if angle > math.radians(10.0):
            twist.linear.x = 0.05
            twist.angular.z = min(self.max_ang, self.goal_turn_gain * angle)
        elif angle < math.radians(-10.0):
            twist.linear.x = 0.05
            twist.angular.z = max(-self.max_ang, self.goal_turn_gain * angle)
        else:
            twist.linear.x = self.goal_forward_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def return_init_turn_after_hit_step(self):
        done = self.perform_turn_step()
        if done:
            self.state = 'RETURN_BOUNDARY_FOLLOW'
            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.get_logger().info('Completed return obstacle turn. Entering RETURN_BOUNDARY_FOLLOW.')

    def return_boundary_follow_step(self):
        self.consume_forward_increment()

        distance = math.hypot(self.home_x - self.est_x, self.home_y - self.est_y)
        angle = self.wrap_angle(math.atan2(self.home_y - self.est_y, self.home_x - self.est_x) - self.yaw)

        if distance <= self.return_home_tolerance:
            self.publish_stop()
            self.est_x = self.home_x
            self.est_y = self.home_y
            self.start_turn_absolute(self.ray_yaw)
            self.state = 'HOME_ALIGN_TO_RAY'
            self.get_logger().info('Returned to centre from boundary follow. Re-aligning to previous ray yaw.')
            return

        sqr = math.hypot(self.hit_x - self.est_x, self.hit_y - self.est_y)
        if sqr > self.MOVEAWAY:
            self.moved = True

        if distance < self.mindist:
            self.mindist = distance

        freespacetogoal = self.get_range_at_bearing(angle)
        front = self.front_distance()
        left = self.get_range_at_bearing(1.571)

        self.get_logger().info(
            f'RETURN_BOUNDARY: d_home={distance:.2f} F={freespacetogoal:.2f} '
            f'min={self.mindist:.2f} left={left:.2f} front={front:.2f}'
        )

        if self.moved and distance - freespacetogoal <= self.mindist - self.STEP:
            self.publish_stop()
            self.leave_points.append((self.est_x, self.est_y))
            self.state = 'RETURN_GO_HOME'
            self.get_logger().info('RETURN LEAVE boundary and resume home seeking.')
            return

        twist = Twist()

        if front < self.WALL_FRONT_BLOCK:
            twist.linear.x = 0.0
            twist.angular.z = -self.max_ang
            self.cmd_pub.publish(twist)
            return

        err = left - self.WALLDIST

        if err > 0.20:
            twist.linear.x = 0.10
            twist.angular.z = 0.6
        elif err > 0.01:
            twist.linear.x = 0.10
            twist.angular.z = 0.15
        elif err < -0.05:
            twist.linear.x = 0.10
            twist.angular.z = -0.6
        elif err < -0.005:
            twist.linear.x = 0.10
            twist.angular.z = -0.15
        else:
            twist.linear.x = 0.10
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def home_align_to_ray_step(self):
        done = self.perform_turn_step()
        if done:
            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.start_turn_absolute(self.wrap_angle(self.ray_yaw + self.center_scan_step))
            self.state = 'CENTER_ROTATE_5'
            self.get_logger().info('Aligned to previous ray yaw. Rotating 5 degrees ACW before resuming scan.')

    def center_rotate_5_step(self):
        done = self.perform_turn_step()
        if done:
            self.publish_stop()
            self.center_scan_accumulated += self.center_scan_step
            self.center_scan_prev_yaw = self.yaw
            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.reset_distbug_memory()
            self.ray_yaw = None
            self.ray_target_x = None
            self.ray_target_y = None
            self.ray_distance = 0.0

            if self.center_scan_accumulated >= self.center_scan_total:
                self.post_scan_delay_start = self.get_clock().now().nanoseconds / 1e9
                self.state = 'POST_SCAN_DELAY'
                self.get_logger().info(
                    f'Centre 360 scan complete after centre step. Landmark photos saved: {self.ray_photo_count}. '
                    f'Waiting {self.post_scan_delay_s:.1f}s before waypoint following.'
                )
            else:
                self.state = 'CENTER_SCAN'
                self.get_logger().info(
                    f'Resuming centre scan. Accumulated centre rotation = '
                    f'{math.degrees(self.center_scan_accumulated):.1f}/360.0 deg.'
                )

    def post_scan_delay_step(self):
        self.publish_stop()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.post_scan_delay_start is None:
            self.post_scan_delay_start = now
            return

        if now - self.post_scan_delay_start >= self.post_scan_delay_s:
            self.mapping_complete = True
            self.publish_stop()
            self.state = 'WAIT_FOR_WAYPOINT_COMMAND'
            self.get_logger().info(
                'Post-scan delay complete. Mapping is done. Waiting for UI Go To Waypoints command.'
            )

            if self.waypoint_start_requested:
                self.start_waypoint_travel_from_landmarks()

    # Main DistBug logic
    def go_to_goal_step(self, goalx: float, goaly: float):
        self.consume_forward_increment()

        if self.check_point(self.est_x, self.est_y, goalx, goaly):
            self.publish_stop()

            if self.returning_to_origin_after_waypoints:
                self.final_target_yaw = math.radians(self.origin_final_yaw_deg)
                self.start_turn_absolute(self.final_target_yaw)
                self.state = 'FINAL_ALIGN_ORIGIN'

                self.get_logger().info(
                    f'Origin/home reached at ({self.est_x:.3f}, {self.est_y:.3f}), '
                    f'now aligning yaw to {self.origin_final_yaw_deg:.1f} deg'
                )
                return

            _, _, final_yaw_deg = self.waypoints[self.current_goal_index]
            self.final_target_yaw = math.radians(final_yaw_deg)
            self.start_turn_absolute(self.final_target_yaw)
            self.state = 'FINAL_ALIGN'

            self.get_logger().info(
                f'POSITION of waypoint {self.current_goal_index + 1}/{len(self.waypoints)} reached '
                f'at ({self.est_x:.3f}, {self.est_y:.3f}), now aligning yaw'
            )
            return

        distance, rot = self.get_relative_goal_location(goalx, goaly)
        freespacetogoal = self.get_range_at_bearing(rot)
        front = self.front_distance()

        if self.returning_to_origin_after_waypoints:
            label_text = "origin/home"
        else:
            label_text = f"wp={self.current_goal_index + 1}/{len(self.waypoints)}"

        self.get_logger().info(
            f'GO_TO_GOAL: {label_text} '
            f'est=({self.est_x:.3f},{self.est_y:.3f}) '
            f'd={distance:.3f} rot={math.degrees(rot):.2f}deg '
            f'front={front:.3f} F={freespacetogoal:.3f}'
        )

        # E-STOP during waypoint approach only.
        # The tracker is active only when this is a real waypoint and the robot
        # is already pointing mostly toward it, which matches the straight-drive
        # branch below. This avoids false E-STOPS while turning or returning home.
        if not self.returning_to_origin_after_waypoints and abs(rot) <= math.radians(10.0):
            estop_distance = self.min_range_in_window(0.0, self.estop_half_fov)
            self.get_logger().info(
                f'ESTOP WATCH: waypoint={self.current_goal_index + 1}/{len(self.waypoints)} '
                f'estop5deg={estop_distance:.3f}m previous={self.estop_previous_distance}'
            )
            if self.check_estop_during_approach('WAYPOINT_APPROACH', estop_distance):
                return
        else:
            self.reset_estop_tracker()

        if front < self.SAFEDISTANCE or freespacetogoal < self.SAFEDISTANCE:
            self.hit_x = self.est_x
            self.hit_y = self.est_y
            self.hit_yaw = self.yaw
            self.mindist = distance
            self.moved = False

            if not self.seen_before(self.est_x, self.est_y):
                self.hit_points.append((self.est_x, self.est_y))
                self.get_logger().info(
                    f'ADDED HIT POINT at ({self.est_x:.3f}, {self.est_y:.3f})'
                )

            self.publish_stop()
            self.start_turn_relative(-math.pi / 2.5)
            self.state = 'INIT_TURN_AFTER_HIT'

            self.get_logger().info(
                f'HIT: hit=({self.hit_x:.3f},{self.hit_y:.3f}) '
                f'mindist={self.mindist:.3f}'
            )
            return

        twist = Twist()
        if rot > math.radians(10.0):
            twist.linear.x = 0.05
            twist.angular.z = min(self.max_ang, self.goal_turn_gain * rot)
        elif rot < math.radians(-10.0):
            twist.linear.x = 0.05
            twist.angular.z = max(-self.max_ang, self.goal_turn_gain * rot)
        else:
            twist.linear.x = self.goal_forward_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def init_turn_after_hit_step(self):
        done = self.perform_turn_step()
        if done:
            self.state = 'BOUNDARY_FOLLOW'
            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.get_logger().info('Completed 60deg hit turn. Entering BOUNDARY_FOLLOW.')

    def boundary_follow_step(self, goalx: float, goaly: float):
        self.reset_estop_tracker()
        self.consume_forward_increment()

        if self.check_point(self.est_x, self.est_y, goalx, goaly):
            self.publish_stop()

            if self.returning_to_origin_after_waypoints:
                self.final_target_yaw = math.radians(self.origin_final_yaw_deg)
                self.start_turn_absolute(self.final_target_yaw)
                self.state = 'FINAL_ALIGN_ORIGIN'

                self.get_logger().info(
                    f'Origin/home reached at ({self.est_x:.3f}, {self.est_y:.3f}) '
                    f'from boundary follow, now aligning yaw to {self.origin_final_yaw_deg:.1f} deg'
                )
                return

            _, _, final_yaw_deg = self.waypoints[self.current_goal_index]
            self.final_target_yaw = math.radians(final_yaw_deg)
            self.start_turn_absolute(self.final_target_yaw)
            self.state = 'FINAL_ALIGN'

            self.get_logger().info(
                f'POSITION of waypoint {self.current_goal_index + 1}/{len(self.waypoints)} reached '
                f'at ({self.est_x:.3f}, {self.est_y:.3f}) from boundary follow, now aligning yaw'
            )
            return

        sqr = math.hypot(self.hit_x - self.est_x, self.hit_y - self.est_y)
        if sqr > self.MOVEAWAY:
            self.moved = True

        if self.moved and self.seen_before(self.est_x, self.est_y):
            if not self.check_point(self.est_x, self.est_y, self.hit_x, self.hit_y):
                self.publish_stop()
                self.finished = True
                self.get_logger().warn('GOAL UNREACHABLE: revisited a previous hit point.')
                self.save_results()
                return

        distance, angle = self.get_relative_goal_location(goalx, goaly)

        if distance < self.mindist:
            self.mindist = distance

        freespacetogoal = self.get_range_at_bearing(angle)
        front = self.front_distance()
        left = self.get_range_at_bearing(1.571)

        if self.returning_to_origin_after_waypoints:
            label_text = "origin/home"
        else:
            label_text = f"wp={self.current_goal_index + 1}/{len(self.waypoints)}"

        self.get_logger().info(
            f'BOUNDARY: {label_text} '
            f'd={distance:.3f} F={freespacetogoal:.3f} '
            f'min={self.mindist:.3f} left={left:.3f} front={front:.3f} '
            f'bearing={angle:.2f} '
            f'expr={distance - freespacetogoal:.3f} <= {self.mindist - self.STEP:.3f}'
        )

        if self.moved and distance - freespacetogoal <= self.mindist - self.STEP:
            self.publish_stop()
            self.leave_points.append((self.est_x, self.est_y))
            self.state = 'GO_TO_GOAL'
            self.get_logger().info(
                f'LEAVE boundary and resume goal-seeking '
                f'(left={left:.3f}, F={freespacetogoal:.3f})'
            )
            return

        twist = Twist()

        if front < self.WALL_FRONT_BLOCK:
            twist.linear.x = 0.0
            twist.angular.z = -self.max_ang
            self.cmd_pub.publish(twist)
            return

        err = left - self.WALLDIST

        if err > 0.20:
            twist.linear.x = 0.10
            twist.angular.z = 0.6
        elif err > 0.01:
            twist.linear.x = 0.10
            twist.angular.z = 0.15
        elif err < -0.05:
            twist.linear.x = 0.10
            twist.angular.z = -0.6
        elif err < -0.005:
            twist.linear.x = 0.10
            twist.angular.z = -0.15
        else:
            twist.linear.x = 0.10
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def final_align_step(self):
        done = self.perform_turn_step()
        if done:
            self.publish_stop()

            self.completed_waypoint_points.append(
                (self.est_x, self.est_y, self.yaw, self.current_goal_index + 1)
            )

            self.get_logger().info(
                f'Waypoint {self.current_goal_index + 1}/{len(self.waypoints)} fully completed at '
                f'({self.est_x:.3f}, {self.est_y:.3f}), '
                f'yaw={math.degrees(self.yaw):.2f} deg'
            )

            self.save_waypoint_photo(self.current_goal_index + 1)

            self.current_goal_index += 1
            self.reset_distbug_memory()

            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.final_target_yaw = None

            if self.current_goal_index >= len(self.waypoints):
                self.start_return_to_origin_after_waypoints()
            else:
                self.state = 'GO_TO_GOAL'

    def final_align_origin_step(self):
        done = self.perform_turn_step()
        if done:
            self.publish_stop()

            self.completed_waypoint_points.append(
                (self.est_x, self.est_y, self.yaw, 0)
            )

            self.get_logger().info(
                f'Returned home to origin and fully completed mission at '
                f'({self.est_x:.3f}, {self.est_y:.3f}), '
                f'yaw={math.degrees(self.yaw):.2f} deg'
            )

            self.returning_to_origin_after_waypoints = False
            self.finished = True

            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.final_target_yaw = None

            self.save_results()

    # Main control loop
    def control_loop(self):
        if self.estop_triggered or self.state == 'ESTOP':
            self.publish_stop()
            return

        if self.finished:
            self.publish_stop()
            return

        if self.est_x is None or self.est_y is None or self.yaw is None or self.scan is None:
            return

        self.record_path_point()

        mapping_or_wait_states = {
            'CENTER_SCAN', 'RAY_APPROACH_OBJECT', 'RAY_CONTINUE',
            'RAY_INIT_TURN_AFTER_OBJECT', 'RAY_BOUNDARY_FOLLOW',
            'RETURN_GO_HOME', 'RETURN_INIT_TURN_AFTER_HIT',
            'RETURN_BOUNDARY_FOLLOW', 'HOME_ALIGN_TO_RAY',
            'CENTER_ROTATE_5', 'POST_SCAN_DELAY',
            'WAIT_FOR_WAYPOINT_COMMAND',
        }
        if self.waypoint_start_requested and self.state in mapping_or_wait_states:
            if self.try_start_waypoint_travel_if_available():
                return

        if self.state == 'CENTER_SCAN':
            self.center_scan_step_method()
            return

        if self.state == 'RAY_APPROACH_OBJECT':
            self.ray_approach_object_step()
            return

        if self.state == 'RAY_CONTINUE':
            self.ray_continue_step()
            return

        if self.state == 'RAY_INIT_TURN_AFTER_OBJECT':
            self.ray_init_turn_after_object_step()
            return

        if self.state == 'RAY_BOUNDARY_FOLLOW':
            self.ray_boundary_follow_step()
            return

        if self.state == 'RETURN_GO_HOME':
            self.return_go_home_step()
            return

        if self.state == 'RETURN_INIT_TURN_AFTER_HIT':
            self.return_init_turn_after_hit_step()
            return

        if self.state == 'RETURN_BOUNDARY_FOLLOW':
            self.return_boundary_follow_step()
            return

        if self.state == 'HOME_ALIGN_TO_RAY':
            self.home_align_to_ray_step()
            return

        if self.state == 'CENTER_ROTATE_5':
            self.center_rotate_5_step()
            return

        if self.state == 'POST_SCAN_DELAY':
            self.post_scan_delay_step()
            return

        if self.state == 'WAIT_FOR_WAYPOINT_COMMAND':
            self.publish_stop()
            if self.waypoint_start_requested:
                self.start_waypoint_travel_from_landmarks()
            return

        if self.current_goal_index >= len(self.waypoints) and not self.returning_to_origin_after_waypoints:
            self.start_return_to_origin_after_waypoints()
            return

        if self.returning_to_origin_after_waypoints:
            goalx = self.origin_goal_x
            goaly = self.origin_goal_y
        else:
            goalx, goaly, _ = self.waypoints[self.current_goal_index]

        if self.state == 'GO_TO_GOAL':
            self.go_to_goal_step(goalx, goaly)
            return

        if self.state == 'INIT_TURN_AFTER_HIT':
            self.init_turn_after_hit_step()
            return

        if self.state == 'BOUNDARY_FOLLOW':
            self.boundary_follow_step(goalx, goaly)
            return

        if self.state == 'FINAL_ALIGN':
            self.final_align_step()
            return

        if self.state == 'FINAL_ALIGN_ORIGIN':
            self.final_align_origin_step()
            return


def main(args=None):
    rclpy.init(args=args)
    node = DistBugController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.save_results()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
