import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class PioneerController(Node):
    def __init__(self):
        super().__init__('local_controller')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer runs the control loop 10 times a second
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.obstacle_danger = False

        # THE PATH (Waypoints)
        # Start at 0,0. Cone is at 5,0. Cylinder is at 10,0.
        self.waypoints = [
            (3.0, 0.0),   # Drive straight towards cone
            (5.0, 2.0),   # Hook LEFT around the cone
            (8.0, 0.0),   # Come back to the center line
            (9.0, 0.0)    # Stop 1 meter before the cylinder
        ]
        self.current_wp_index = 0

    def odom_callback(self, msg):
        # Get X and Y
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert Quaternion to Euler Yaw (Z-axis rotation)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # Look at the middle 100 Lidar rays (directly in front)
        front_rays = msg.ranges[270:370]
        min_distance = min(front_rays)
        
        # If something is closer than 0.8 meters, panic!
        if min_distance < 0.8:
            self.obstacle_danger = True
        else:
            self.obstacle_danger = False

    def control_loop(self):
        msg = Twist()

        # Check if mission is complete
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('Destination Reached. Stopping.')
            self.cmd_pub.publish(msg) # Publishes 0 velocity
            return

        # OBSTACLE OVERRIDE 
        if self.obstacle_danger:
            self.get_logger().warning('Unseen Obstacle! Halting and turning...')
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Spin left to find a clear path
            self.cmd_pub.publish(msg)
            return

        # WAYPOINT NAVIGATION MATH
        target_x, target_y = self.waypoints[self.current_wp_index]
        
        # Calculate distance and angle to target
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)

        # Normalize the angle difference so the robot turns the shortest way
        angle_diff = angle_to_target - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        if distance < 0.3:
            # We are within 30cm of the waypoint, load the next one!
            self.get_logger().info(f'Reached Waypoint {self.current_wp_index + 1}')
            self.current_wp_index += 1
        else:
            # Standard driving logic
            if abs(angle_diff) > 0.15:
                # If we are facing the wrong way, pivot on the spot
                msg.linear.x = 0.0
                msg.angular.z = 0.4 if angle_diff > 0 else -0.4
            else:
                # If we are facing the target, drive forward!
                msg.linear.x = 0.4
                msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PioneerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
