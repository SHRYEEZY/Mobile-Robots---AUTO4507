#!/usr/bin/env python3
# PART 1 SIMULATION - SHRIRAM NARENDRAN

import math
import csv
import os #file paths
from datetime import datetime #files saved according to time
from typing import List, Tuple, Optional #typing formats for ease of reading

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node #For ros2 node created
from geometry_msgs.msg import Twist #Movement control
from nav_msgs.msg import Odometry #Mainly for linear movement
from sensor_msgs.msg import Imu, LaserScan #Obvious


class DistBugController(Node):
    def __init__(self):
        super().__init__('distbug_controller') #Name of node

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #Velocity commands publisher

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10 #Listens to odom i.e encoder equivalent, but mainly for linear things
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10 #Angular velocity converted to rotation
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10 #360 scan similar to eyesim
        )

        self.timer = self.create_timer(0.05, self.control_loop) #Repeat every 0.05 seconds, very good

        # Hybrid pose estimate constant placeholders below

        self.yaw = None #Angle

        self.odom_x = None
        self.odom_y = None # Yes you use the x and y of odom, but it's mainly just to compare linear movement.
        self.prev_odom_x = None
        self.prev_odom_y = None #Previous perceived position
        self.latest_distance_increment = 0.0

        self.est_x = None
        self.est_y = None #Estimated x and y

        # Scan data placeholder
        self.scan: Optional[LaserScan] = None

        # Waypoints: (x, y, final_angle)

        self.waypoints: List[Tuple[float, float, float]] = [
            (3.0, 0.0, 0.0),
            (2.0, 2.0, 90.0),
            (3.0, 3.0, 0.0),
            (5.0, -1.0, 180.0),
            (0.0, -1.0, 180.0),
            (-2.0, -1.0, 90.0),
            (-2.0, 2.0, 90.0),
            (-2.0, 5.0, 0.0),
            (0.0, 5.0, 0.0),
            (0.0, 0.0, 180.0),
        ]
        
        self.current_goal_index = 0 #Increments once goal position achieved

        # Initial states set
        self.state = 'GO_TO_GOAL'
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
        self.yaw_tolerance = math.radians(0.9) #self.yaw_tolerance_deg = 0.9
        self.turn_start_time = None
        self.turn_timeout = None

        # final desired yaw at the end of a way point
        self.final_target_yaw = None

        # Distance keeping and dist bug constants (meters)
        self.STEP = 3.5
        self.SAFEDISTANCE = 0.45 #Goal finding safe distance
        self.MOVEAWAY = 0.9 #Only after 90 cm movement good to leave destination.
        self.WALL_FRONT_BLOCK = 0.60 # For front check during boundary travels (safeguards object collisions)
        self.WALLDIST = 1.0 # Boundary following distance (if the safe distance isn't great or is the turn when meeting an object, this gets cooked as it seems)
        self.GOALTOL = 0.03

        # Motion constants
        self.goal_forward_speed = 0.40
        self.goal_turn_gain = 1.8 #Increases base angular turn extent, helps tune beyond initial angular turn bt this value ended up perfectly fine
        self.max_ang = 0.8 #It's a set angle to turn around 45 degree if u detect obstacle DURING boundary, but also to cap goal travelling adjustments
        self.wall_follow_speed = 0.10
        self.min_distance_increment = 0.0005

        # Map / logging settings
        # Adjust these to your actual world size if needed, we will stick with 100x100 base grid. 
        self.map_x_min = -50.0
        self.map_x_max = 50.0
        self.map_y_min = -50.0
        self.map_y_max = 50.0

        self.output_dir = os.path.expanduser("~/ros2_ws")
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
    def wrap_angle(self, angle: float) -> float: #Ensure angle is within 2pi range
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quat_to_yaw(self, q) -> float: #Converts imu readings into heading
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp) 

    def publish_stop(self): 
        self.cmd_pub.publish(Twist()) #Publishes a stop command of 0 velocity to well, stop the bot

    def check_point(self, x: float, y: float, goalx: float, goaly: float) -> bool:
        dx = goalx - x
        dy = goaly - y
        distance = math.hypot(dx, dy)
        return distance <= self.GOALTOL #Checks if goal has been reached given distance from the goal is less than tolerance
        
    def check_pointv2(self, x: float, y: float, goalx: float, goaly: float) -> bool:
        dx = goalx - x
        dy = goaly - y
        distance = math.hypot(dx, dy)
        return distance <= 0.3 #Checks if goal has been reached given distance from the goal is less than tolerance

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

        return distance, rot #What is the relative location of the goal to the pioneer? 

    def reset_distbug_memory(self):
        self.hit_x = None
        self.hit_y = None
        self.hit_yaw = None
        self.mindist = None
        self.moved = False #Done when waypoint reached

    def record_path_point(self):
        if self.est_x is None or self.est_y is None or self.yaw is None:
            return #Is input valid?

        if self.start_point is None:
            self.start_point = (self.est_x, self.est_y) #Save estimated starting position

        if not self.path_points:
            self.path_points.append((self.est_x, self.est_y))
            self.path_records.append((self.est_x, self.est_y, self.yaw, self.state))
            return #Record first point

        last_x, last_y = self.path_points[-1] #Save last point
        if math.hypot(self.est_x - last_x, self.est_y - last_y) >= self.path_log_spacing:
            self.path_points.append((self.est_x, self.est_y))
            self.path_records.append((self.est_x, self.est_y, self.yaw, self.state)) #So long as point is well spaced enough, then record point

    def save_results(self):
        if self.map_saved:
            return

        self.map_saved = True
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        full_png = os.path.join(self.output_dir, f"distbug_map_FULL_{timestamp}.png")
        zoom_png = os.path.join(self.output_dir, f"distbug_map_ZOOM_{timestamp}.png")
        csv_path = os.path.join(self.output_dir, f"distbug_path_{timestamp}.csv") #Create images and csv files holding map and data

        # Write data saved to the csv file, contains cool stuff like position, angle at that position and the state experienced
        
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw_rad", "state"])
            for row in self.path_records:
                writer.writerow(row)

        # Draw map contents
        def draw_map(ax):
            # Path
            if self.path_points:
                xs = [p[0] for p in self.path_points]
                ys = [p[1] for p in self.path_points]
                ax.plot(xs, ys, linewidth=2.5, label="Driven path")

            # Start
            if self.start_point is not None:
                ax.scatter(*self.start_point, s=100, marker='o', label="Start")
                ax.text(*self.start_point, "  Start")

            # Waypoints
            for i, (wx, wy, _) in enumerate(self.waypoints, start=1):
                ax.scatter(wx, wy, s=100, marker='s')
                ax.text(wx, wy+0.2, f"  WP{i}")

            # Hit points
            if self.hit_points:
                hx = [p[0] for p in self.hit_points]
                hy = [p[1] for p in self.hit_points]
                ax.scatter(hx, hy, s=80, marker='x', label="Hits")
                for i, (x, y) in enumerate(self.hit_points, start=1):
                    ax.text(x, y, f"  H{i}")

            # Leave points
            if self.leave_points:
                lx = [p[0] for p in self.leave_points]
                ly = [p[1] for p in self.leave_points]
                ax.scatter(lx, ly, s=80, marker='^', label="Leaves")
                for i, (x, y) in enumerate(self.leave_points, start=1):
                    ax.text(x, y, f"  L{i}")

            # Completed waypoint poses + yaw arrows
            for x, y, yaw, idx in self.completed_waypoint_points:
                # ax.scatter(x, y, s=80, marker='o')
                dx = 0.3 * math.cos(yaw)
                dy = 0.3 * math.sin(yaw) #Gives length for the arrow in respective directions of angle
                ax.arrow(
                    x, y, dx, dy,
                    head_width=0.1,
                    head_length=0.1,
                    length_includes_head=True
                )
                # ax.text(x-0.2, y-0.2, f"  Done{idx}")

            ax.grid(True)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_aspect('equal')

        # FULL MAP CONSTRUCTION
        
        fig1, ax1 = plt.subplots(figsize=(10, 10))
        draw_map(ax1)

        ax1.set_xlim(self.map_x_min, self.map_x_max)
        ax1.set_ylim(self.map_y_min, self.map_y_max)
        ax1.set_title("PIONEER VOYAGE (Full World)")
        ax1.legend()

        plt.tight_layout()
        plt.savefig(full_png, dpi=350)
        plt.close(fig1)

        # ZOOM MAP CONSTRUCTION
        fig2, ax2 = plt.subplots(figsize=(10, 10)) 
        draw_map(ax2) # Redraws previous map

        xs, ys = [], [] #All points are evaluated to xs and ys arraw to evaluate the largest and smallest extents

        if self.path_points:
            xs += [p[0] for p in self.path_points]
            ys += [p[1] for p in self.path_points]

        if self.start_point:
            xs.append(self.start_point[0])
            ys.append(self.start_point[1])

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
        self.odom_y = msg.pose.pose.position.y #Odom is good regarding changes in position, but very bad in the true position idk why. So odom acts similar to encoder in order to get change in motion. 

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
        self.yaw = self.quat_to_yaw(msg.orientation) #Obtain imu feedback

    def scan_callback(self, msg: LaserScan):
        self.scan = msg #Obtain laser feedback

    # Pose estimation
    def consume_forward_increment(self):
        dist_inc = self.latest_distance_increment
        if dist_inc > self.min_distance_increment:
            self.est_x += dist_inc * math.cos(self.yaw)
            self.est_y += dist_inc * math.sin(self.yaw)
        self.latest_distance_increment = 0.0

    # LaserScan helpers
    
    def get_range_at_bearing(self, bearing_rad: float) -> float: #Gets the range at the bearing input, fair enough.
        if self.scan is None:
            return 0.0

        if bearing_rad < self.scan.angle_min or bearing_rad > self.scan.angle_max:
            return self.scan.range_max #Assume object not in range

        idx = int((bearing_rad - self.scan.angle_min) / self.scan.angle_increment) #You see angles of lidar are in increments, not just angles. So take the difference in angle, and divide by step size
        idx = max(0, min(idx, len(self.scan.ranges) - 1)) #Take max of 0, or the minimum of either wt's found at angle or the total minimum found.

        r = self.scan.ranges[idx] #Obtain range at the found id.
        if math.isinf(r) or math.isnan(r):
            return self.scan.range_max #If lidar returns invalid or no object read return not detected
        return r #Return range if it passes thru all 'at

    def min_range_in_window(self, center_rad: float, half_width_rad: float) -> float:
    # Finds the smallest range in window, good to see if an object is about to come. 
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

    def front_distance(self) -> float: #Checks the distance in front, avoiding any collisions
        return self.min_range_in_window(0.0, math.radians(60.0))
        
    #def left_distance(self) -> float: #Checks the distance in left, keeping distance
        #return self.min_range_in_window(math.radians(91.0), math.radians(90.0))

#Below things get less helper and more about physical actions such as turns

    # Turn helpers
    
    def start_turn_relative(self, delta_rad: float): #This is for turning the relative distance, one is a general movement to close down to desired angle difference, another bit is a timer hold to ensure it doesn't take too long.
        self.turn_target_yaw = self.wrap_angle(self.yaw + delta_rad)
        self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

        yaw_error = self.wrap_angle(self.turn_target_yaw - self.yaw)
        ideal_time = abs(yaw_error) / self.angular_speed if self.angular_speed > 0.0 else 0.0
        self.turn_timeout = ideal_time * self.turn_correction

    def start_turn_absolute(self, target_yaw_rad: float):
    #Same as above but to turn to a given angle not a difference.
        self.turn_target_yaw = self.wrap_angle(target_yaw_rad)
        self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

        yaw_error = self.wrap_angle(self.turn_target_yaw - self.yaw)
        ideal_time = abs(yaw_error) / self.angular_speed if self.angular_speed > 0.0 else 0.0
        self.turn_timeout = ideal_time * self.turn_correction

    def perform_turn_step(self) -> bool:
    #The above two functions set the time required for turning and yaw, this executes it
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

    # Main DistBug logic
    
    def go_to_goal_step(self, goalx: float, goaly: float):
        self.consume_forward_increment()

        if self.check_point(self.est_x, self.est_y, goalx, goaly):
        #If we have reached required point
            self.publish_stop()

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
        #We are in going to goal step

        self.get_logger().info(
            f'GO_TO_GOAL: wp={self.current_goal_index + 1}/{len(self.waypoints)} '
            f'est=({self.est_x:.3f},{self.est_y:.3f}) '
            f'd={distance:.3f} rot={math.degrees(rot):.2f}deg '
            f'front={front:.3f} F={freespacetogoal:.3f}'
        )
        
        #If we notice front is cooked as we are heading to a waypoint, we stop and record hit

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

        #Move forward and turn to required point
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

    def init_turn_after_hit_step(self): #Obvious, turn after u hit
        done = self.perform_turn_step()
        if done:
            self.state = 'BOUNDARY_FOLLOW'
            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.get_logger().info('Completed 60deg hit turn. Entering BOUNDARY_FOLLOW.')

    def boundary_follow_step(self, goalx: float, goaly: float):
    #Follow the border, until u are either at destination, back to where u started or u have found the target in sight and can leave
        self.consume_forward_increment()

        if self.check_point(self.est_x, self.est_y, goalx, goaly):
            self.publish_stop()

            _, _, final_yaw_deg = self.waypoints[self.current_goal_index]
            self.final_target_yaw = math.radians(final_yaw_deg)
            self.start_turn_absolute(self.final_target_yaw) #self.yaw_tolerance_deg = 0.9
            self.state = 'FINAL_ALIGN'

            self.get_logger().info(
                f'POSITION of waypoint {self.current_goal_index + 1}/{len(self.waypoints)} reached '
                f'at ({self.est_x:.3f}, {self.est_y:.3f}) from boundary follow, now aligning yaw'
            )
            return

        sqr = math.hypot(self.hit_x - self.est_x, self.hit_y - self.est_y)
        if sqr > self.MOVEAWAY:
            self.moved = True

        if self.moved and self.seen_before(self.est_x, self.est_y):  # If we have moved away and revisited any PREVIOUS hit point, goal is unreachable
            # ignore the current/latest hit point itself
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
        #corner = self.get_range_at_bearing(1.571/2)
        #corner2 = self.get_range_at_bearing(-1.571/2) u see i tried this but then it's not fully necessary. Front handles up to 60 deg. And then corner ensures a catered safeguard against random corners popping out of object. I didn't do this for right so that it didn't risk interacting with something else. 

        self.get_logger().info(
            f'BOUNDARY: wp={self.current_goal_index + 1}/{len(self.waypoints)} '
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

        #If during ur boundary travels, front sensor is triggered u should stop
        if front < self.WALL_FRONT_BLOCK:
            twist.linear.x = 0.0
            twist.angular.z = -self.max_ang
            self.cmd_pub.publish(twist)
            return

        err = left - self.WALLDIST #Follow wall well
        #safe = corner - self.WALLDIST I just haven't done corner2 to be checked cause like, we will always go right of object, don want it to distance keep with some object on the right. 

        #if safe < 0.3:
         #   twist.linear.x = 0.02
          #  twist.angular.z = 0.6
            
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

    def final_align_step(self): #Do ur final align, turn to required yaw, and reset things. 
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

            self.current_goal_index += 1
            self.reset_distbug_memory()

            self.turn_target_yaw = None
            self.turn_start_time = None
            self.turn_timeout = None
            self.final_target_yaw = None

            if self.current_goal_index >= len(self.waypoints):
                self.finished = True
                self.publish_stop()
                self.get_logger().info('All waypoints completed')
                self.save_results()
            else:
                self.state = 'GO_TO_GOAL'


    # The controlling loop: Defines basically a state machine that corresponds to above functions. U basically keep repeating them as required until all waypoints are done
    def control_loop(self):
        if self.finished:
            self.publish_stop()
            return

        if self.est_x is None or self.est_y is None or self.yaw is None or self.scan is None:
            return

        self.record_path_point()

        if self.current_goal_index >= len(self.waypoints):
            self.finished = True
            self.publish_stop()
            self.get_logger().info('All waypoints completed')
            self.save_results()
            return

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

#The ultimate star point, sets the node nd ros up, performs the node, and then saves results and kills it off as well as turning the program off
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
