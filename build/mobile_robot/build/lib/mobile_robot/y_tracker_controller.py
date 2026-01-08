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
        
        # We track Global Pose separately for logging
        self.global_x, self.global_y = 0.0, 0.0
        self.start_x, self.start_y, self.start_yaw = None, None, None
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        
        self.clean_step = 0
        self.fixed_axis_value = 0.0
        self.step_start_pos = 0.0
        self.partition_max_x = 0.0
        
        self.backup_start_pos = None
        self.wait_start_time = None
        self.front_idx = None
        self.left_idx = None

        self.get_logger().info("Robot 1 Ready: Measuring Width...")

    def length_callback(self, msg):
        self.map_length = msg.data
        self.check_start_cleaning()

    def odom_callback(self, msg):
        # Store Global Coordinates for debugging
        self.global_x = msg.pose.pose.position.x
        self.global_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.start_x is None:
            self.start_x, self.start_y, self.start_yaw = self.global_x, self.global_y, yaw

        dx = self.global_x - self.start_x
        dy = self.global_y - self.start_y

        self.current_y = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        self.current_x = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        self.current_yaw = yaw - self.start_yaw

    def compute_indices(self, scan):
        angle_min = scan.angle_min
        inc = scan.angle_increment
        def a2i(a): return int((a - angle_min) / inc)
        self.front_idx = max(0, min(len(scan.ranges)-1, a2i(0.0)))
        self.left_idx = max(0, min(len(scan.ranges)-1, a2i(math.pi/2)))

    def turn_in_place(self, target_yaw):
        cmd = Twist()
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        diff = math.atan2(math.sin(target_yaw - cyaw), math.cos(target_yaw - cyaw))

        if abs(diff) > 0.02:
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * diff))
            self.cmd_pub.publish(cmd)
            return False
        else:
            self.stop_robot()
            return True

    def move_backward(self, distance):
        if self.backup_start_pos is None:
            self.backup_start_pos = (self.current_x, self.current_y)
            return False
        dist_moved = math.sqrt((self.current_x - self.backup_start_pos[0])**2 + (self.current_y - self.backup_start_pos[1])**2)
        if dist_moved < distance:
            cmd = Twist(); cmd.linear.x = -0.2
            self.cmd_pub.publish(cmd)
            return False
        else:
            self.stop_robot()
            self.backup_start_pos = None
            return True

    def wait_in_place(self, duration):
        if self.wait_start_time is None:
            self.wait_start_time = self.get_clock().now()
            self.stop_robot()
            return False
        elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
        if elapsed < duration:
            self.stop_robot()
            return False
        else:
            self.wait_start_time = None
            return True

    def move_straight_locked(self, target_yaw, axis_to_hold, target_val):
        cmd = Twist(); cmd.linear.x = 0.4
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        yaw_err = math.atan2(math.sin(target_yaw - cyaw), math.cos(target_yaw - cyaw))
        pos_err = target_val - (self.current_x if axis_to_hold == 'x' else self.current_y)
        
        K_YAW, K_POS = 2.0, 1.0
        correction = K_YAW * yaw_err
        if abs(target_yaw) < 0.1: correction += K_POS * pos_err
        elif abs(target_yaw - math.pi) < 0.1 or abs(target_yaw + math.pi) < 0.1: correction -= K_POS * pos_err
        elif abs(target_yaw + math.pi/2) < 0.1: correction += K_POS * pos_err
        elif abs(target_yaw - math.pi/2) < 0.1: correction -= K_POS * pos_err

        cmd.angular.z = max(-0.5, min(0.5, correction))
        self.cmd_pub.publish(cmd)

    def lidar_callback(self, scan):
        if self.front_idx is None: self.compute_indices(scan)
        ranges = np.where(np.isfinite(scan.ranges), scan.ranges, 12.0)
        front_dist = ranges[self.front_idx]

        if self.state == "MEASURING":
            if front_dist < 0.6:
                self.map_width = abs(self.current_y) + front_dist
                self.get_logger().info(f"COORD: Measurement Finished at [{self.global_x:.2f}, {self.global_y:.2f}]")
                self.dim_pub.publish(Float32(data=float(self.map_width)))
                self.state = "BACKING_AFTER_MEASURE"
                return
            cmd = Twist(); cmd.linear.x = 0.4; cmd.angular.z = -1.5 * (0.5 - ranges[self.left_idx]); self.cmd_pub.publish(cmd)

        elif self.state == "BACKING_AFTER_MEASURE":
            if self.move_backward(1.0): self.state = "WAITING"

        elif self.state == "WAITING":
            self.check_start_cleaning()

        elif self.state == "CLEANING":
            # 0. Start -> Turn East
            if self.clean_step == 0:
                if self.turn_in_place(-math.pi/2):
                    self.get_logger().info(f"COORD: Turn East at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.fixed_axis_value = self.current_y; self.step_start_pos = self.current_x; self.clean_step = 1

            # 1. Move East (Toward Partition Line)
            elif self.clean_step == 1:
                if abs(self.current_x) >= self.partition_max_x:
                    self.get_logger().info(f"COORD: Reached Partition Wall at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.stop_robot(); self.clean_step = 2
                elif abs(self.current_x - self.step_start_pos) < 2.0:
                    self.move_straight_locked(-math.pi/2, 'y', self.fixed_axis_value)
                else:
                    self.get_logger().info(f"COORD: Lane Shift Limit reached at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.clean_step = 2 

            # 2. Wait 
            elif self.clean_step == 2:
                if self.wait_in_place(2.0): self.clean_step = 3

            # 3. Turn South (PI)
            elif self.clean_step == 3:
                if self.turn_in_place(math.pi):
                    self.get_logger().info(f"COORD: Turn South at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.fixed_axis_value = self.current_x; self.clean_step = 4

            # 4. Move South
            elif self.clean_step == 4:
                if self.current_y > 0.5 and front_dist > 0.6:
                    self.move_straight_locked(math.pi, 'x', self.fixed_axis_value)
                else:
                    self.get_logger().info(f"COORD: South Wall Stop at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.clean_step = 5

            # 5. Back Up
            elif self.clean_step == 5:
                if self.move_backward(1.0): self.clean_step = 6

            # 6. Turn East again
            elif self.clean_step == 6:
                if self.turn_in_place(-math.pi/2):
                    self.get_logger().info(f"COORD: Turn East at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.fixed_axis_value = self.current_y; self.step_start_pos = self.current_x; self.clean_step = 7

            # 7. Move East (Toward Partition Line)
            elif self.clean_step == 7:
                if abs(self.current_x) >= self.partition_max_x:
                    self.get_logger().info(f"COORD: Reached Partition Wall at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.stop_robot(); self.clean_step = 8
                elif abs(self.current_x - self.step_start_pos) < 2.0:
                    self.move_straight_locked(-math.pi/2, 'y', self.fixed_axis_value)
                else:
                    self.get_logger().info(f"COORD: Lane Shift Limit reached at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.clean_step = 8

            # 8. Wait
            elif self.clean_step == 8:
                if self.wait_in_place(2.0): self.clean_step = 9

            # 9. Turn North (0.0)
            elif self.clean_step == 9:
                if self.turn_in_place(0.0):
                    self.get_logger().info(f"COORD: Turn North at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.fixed_axis_value = self.current_x; self.clean_step = 10

            # 10. Move North
            elif self.clean_step == 10:
                if self.current_y < (self.map_width - 0.5) and front_dist > 0.6:
                    self.move_straight_locked(0.0, 'x', self.fixed_axis_value)
                else:
                    self.get_logger().info(f"COORD: North Wall Stop at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.clean_step = 11
            
            # 11. Final Back up
            elif self.clean_step == 11:
                if self.move_backward(1.0):
                    self.get_logger().info(f"COORD: Loop Reset at [{self.global_x:.2f}, {self.global_y:.2f}]")
                    self.clean_step = 0 

    def check_start_cleaning(self):
        if self.state == "WAITING" and self.map_length is not None:
            self.state = "CLEANING"
            self.partition_max_x = self.map_length / 2.0
            self.get_logger().info(f"R1 Partition Boundary: {self.partition_max_x:.2f}")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YAxisTrackerAndCleaner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()