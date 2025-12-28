import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math

class YAxisTrackerAndCleaner(Node):
    def __init__(self):
        super().__init__('y_axis_tracker')

        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.dim_pub = self.create_publisher(Float32, '/map/width', 10)
        self.length_sub = self.create_subscription(Float32, '/map/length', self.length_callback, 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.lidar_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/robot1/odom1', self.odom_callback, qos_profile_sensor_data)

        self.state = "MEASURING"
        self.map_width = None
        self.map_length = None
        
        self.start_x, self.start_y, self.start_yaw = None, None, None
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        
        # P-Controller Variables
        self.clean_step = 0
        self.fixed_axis_value = 0.0 # The coordinate to lock onto
        self.step_start_pos = 0.0   # To track distance moved
        self.partition_max_x = 0.0
        
        self.front_idx = None
        self.left_idx = None

        self.get_logger().info("Robot 1 (Y-Tracker) Ready with P-Control.")

    def length_callback(self, msg):
        self.map_length = msg.data
        self.check_start_cleaning()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.start_x is None:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.start_yaw = yaw

        # Transform Global -> Local Frame (Start = 0,0)
        dx = msg.pose.pose.position.x - self.start_x
        dy = msg.pose.pose.position.y - self.start_y

        self.current_y = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        self.current_x = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        self.current_yaw = yaw - self.start_yaw

    def compute_indices(self, scan):
        angle_min = scan.angle_min
        inc = scan.angle_increment
        def a2i(a): return int((a - angle_min) / inc)
        self.front_idx = max(0, min(len(scan.ranges)-1, a2i(0.0)))
        self.left_idx = max(0, min(len(scan.ranges)-1, a2i(math.pi/2)))

    # --- P-CONTROLLER LOGIC ---
    def turn_in_place(self, target_yaw):
        """Rotates until facing target_yaw within strict tolerance."""
        cmd = Twist()
        cmd.linear.x = 0.0
        
        # Normalize angles
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        diff = target_yaw - cyaw
        diff = math.atan2(math.sin(diff), math.cos(diff))

        if abs(diff) > 0.02: # Strict tolerance (approx 1 degree)
            cmd.angular.z = 0.3 if diff > 0 else -0.3
            self.cmd_pub.publish(cmd)
            return False # Still turning
        else:
            self.cmd_pub.publish(cmd) # Stop
            return True # Done

    def move_straight_locked(self, target_yaw, axis_to_hold, target_val):
        """
        Moves forward while steering to keep 'axis_to_hold' == 'target_val'.
        """
        cmd = Twist()
        cmd.linear.x = 0.4 # Slower speed for better control

        # 1. Heading Error
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        yaw_err = target_yaw - cyaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        # 2. Position Error (Cross-Track Error)
        pos_err = 0.0
        if axis_to_hold == 'x':
            pos_err = target_val - self.current_x
        elif axis_to_hold == 'y':
            pos_err = target_val - self.current_y

        # 3. Control Gains
        K_YAW = 2.0
        K_POS = 1.0

        # 4. Calculate Correction
        correction = K_YAW * yaw_err

        # Adjust steering based on direction to fix position error
        # Logic: If I need to increase X, which way do I turn?
        if abs(target_yaw - 0.0) < 0.1: # Moving North
            # If X is too low (positive error), turn Left (+Z) to go +X
            correction += K_POS * pos_err 
        elif abs(target_yaw - math.pi) < 0.1 or abs(target_yaw + math.pi) < 0.1: # Moving South
            # If X is too low (pos error), turn Right (-Z) because we are backing/facing South
            correction -= K_POS * pos_err
        elif abs(target_yaw + math.pi/2) < 0.1: # Moving East (-PI/2)
            # If Y is too low (pos error), turn Left (+Z) to go +Y (North is Left of East)
            correction += K_POS * pos_err
        elif abs(target_yaw - math.pi/2) < 0.1: # Moving West
            # If Y is too low, turn Right (-Z)
            correction -= K_POS * pos_err

        cmd.angular.z = max(-0.5, min(0.5, correction))
        self.cmd_pub.publish(cmd)

    def lidar_callback(self, scan):
        if self.front_idx is None: self.compute_indices(scan)
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 12.0)
        front_dist = ranges[self.front_idx]

        if self.state == "MEASURING":
            if front_dist < 0.6:
                self.map_width = abs(self.current_y) + front_dist
                msg = Float32(); msg.data = float(self.map_width); self.dim_pub.publish(msg)
                self.state = "WAITING"
                self.stop_robot()
                return
            
            # Wall Follower (Measuring Phase)
            left = ranges[self.left_idx]
            cmd = Twist()
            if left > 0.6: cmd.linear.x = 0.3; cmd.angular.z = 0.3
            elif left < 0.4: cmd.linear.x = 0.3; cmd.angular.z = -0.3
            else: cmd.linear.x = 0.9; cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

        elif self.state == "WAITING":
            self.check_start_cleaning()

        elif self.state == "CLEANING":
            # --- PRECISE S-SHAPE ---
            
            # 0. Turn East (-PI/2)
            if self.clean_step == 0:
                if self.turn_in_place(-math.pi/2):
                    self.fixed_axis_value = self.current_y # LOCK Y
                    self.step_start_pos = self.current_x
                    self.clean_step = 1

            # 1. Move East 2m
            elif self.clean_step == 1:
                if abs(self.current_x) >= self.partition_max_x: # Boundary Check
                    self.stop_robot(); return
                
                if abs(self.current_x - self.step_start_pos) < 2.0:
                    self.move_straight_locked(-math.pi/2, 'y', self.fixed_axis_value)
                else:
                    self.clean_step = 2

            # 2. Turn South (PI)
            elif self.clean_step == 2:
                if self.turn_in_place(math.pi):
                    self.fixed_axis_value = self.current_x # LOCK X
                    self.clean_step = 3

            # 3. Move South (Down)
            elif self.clean_step == 3:
                # Move until near Y=0 or Wall
                if self.current_y > 0.5 and front_dist > 0.6:
                    self.move_straight_locked(math.pi, 'x', self.fixed_axis_value)
                else:
                    self.clean_step = 4

            # 4. Turn East (-PI/2)
            elif self.clean_step == 4:
                if self.turn_in_place(-math.pi/2):
                    self.fixed_axis_value = self.current_y # LOCK Y
                    self.step_start_pos = self.current_x
                    self.clean_step = 5

            # 5. Move East 2m
            elif self.clean_step == 5:
                if abs(self.current_x) >= self.partition_max_x:
                    self.stop_robot(); return

                if abs(self.current_x - self.step_start_pos) < 2.0:
                    self.move_straight_locked(-math.pi/2, 'y', self.fixed_axis_value)
                else:
                    self.clean_step = 6

            # 6. Turn North (0.0)
            elif self.clean_step == 6:
                if self.turn_in_place(0.0):
                    self.fixed_axis_value = self.current_x # LOCK X
                    self.clean_step = 7

            # 7. Move North (Up)
            elif self.clean_step == 7:
                if self.current_y < (self.map_width - 0.5) and front_dist > 0.6:
                    self.move_straight_locked(0.0, 'x', self.fixed_axis_value)
                else:
                    self.clean_step = 0 # Loop back to step 0

    def check_start_cleaning(self):
        if self.state == "WAITING" and self.map_length is not None:
            self.state = "CLEANING"
            self.partition_max_x = self.map_length / 2.0
            self.get_logger().info(f"R1 Cleaning Phase. Zone X: 0 - {self.partition_max_x:.2f}")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YAxisTrackerAndCleaner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()