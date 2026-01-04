import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math

class XAxisTrackerAndCleaner(Node):
    def __init__(self):
        super().__init__('x_axis_tracker')

        self.ORIGIN_OFFSET = 1.0
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.dim_pub = self.create_publisher(Float32, '/map/length', 10)
        self.width_sub = self.create_subscription(Float32, '/map/width', self.width_callback, 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/robot2/scan', self.lidar_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/robot2/odom1', self.odom_callback, qos_profile_sensor_data)

        self.state = "MEASURING"
        self.map_length = None
        self.map_width = None
        
        self.start_x, self.start_y, self.start_yaw = None, None, None
        self.current_x, self.current_y, self.current_yaw = self.ORIGIN_OFFSET, 0.0, 0.0
        
        self.clean_step = 0
        self.fixed_axis_value = 0.0
        self.step_start_pos = 0.0
        self.partition_min_x = 0.0
        
        self.backup_start_pos = None
        self.wait_start_time = None
        self.current_front_dist = 10.0
        
        self.front_idx = None
        self.right_idx = None
        self.get_logger().info("Robot 2 Ready: Measuring Length...")

    def width_callback(self, msg):
        self.map_width = msg.data
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
        dx = msg.pose.pose.position.x - self.start_x
        dy = msg.pose.pose.position.y - self.start_y
        self.current_x = (dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)) + self.ORIGIN_OFFSET
        self.current_y = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        self.current_yaw = yaw - self.start_yaw

    def compute_indices(self, scan):
        angle_min = scan.angle_min
        inc = scan.angle_increment
        def a2i(a): return int((a - angle_min) / inc)
        self.front_idx = max(0, min(len(scan.ranges)-1, a2i(0.0)))
        self.right_idx = max(0, min(len(scan.ranges)-1, a2i(-math.pi/2)))

    def turn_in_place(self, target_yaw):
        cmd = Twist()
        cmd.linear.x = 0.0
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        diff = target_yaw - cyaw
        diff = math.atan2(math.sin(diff), math.cos(diff))

        if abs(diff) > 0.005:
            speed = 1.0 * diff
            min_speed = 0.2
            if speed > 0: speed = max(min_speed, min(0.5, speed))
            else: speed = min(-min_speed, max(-0.5, speed))
            cmd.angular.z = speed
            self.cmd_pub.publish(cmd)
            return False
        else:
            self.cmd_pub.publish(Twist()) 
            return True

    def move_backward(self, distance):
        if self.backup_start_pos is None: self.backup_start_pos = (self.current_x, self.current_y); return False
        dx = self.current_x - self.backup_start_pos[0]; dy = self.current_y - self.backup_start_pos[1]
        dist_moved = math.sqrt(dx*dx + dy*dy)
        if dist_moved < distance: cmd = Twist(); cmd.linear.x = -0.2; self.cmd_pub.publish(cmd); return False
        else: self.cmd_pub.publish(Twist()); self.backup_start_pos = None; return True

    def wait_in_place(self, duration):
        if self.wait_start_time is None: self.wait_start_time = self.get_clock().now(); self.cmd_pub.publish(Twist()); return False
        elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
        if elapsed < duration: self.cmd_pub.publish(Twist()); return False
        else: self.wait_start_time = None; return True

    def move_straight_locked(self, target_yaw, axis_to_hold, target_val):
        cmd = Twist()
        cmd.linear.x = 0.4
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        
        yaw_err = target_yaw - cyaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        pos_err = 0.0
        if axis_to_hold == 'x': pos_err = target_val - self.current_x
        elif axis_to_hold == 'y': pos_err = target_val - self.current_y

        K_YAW = 2.0; K_POS = 1.0
        if self.current_front_dist < 1.0: K_POS = 0.0 

        correction = K_YAW * yaw_err

        if abs(target_yaw - math.pi/2) < 0.1: correction -= K_POS * pos_err
        elif abs(target_yaw) > 3.0: correction += K_POS * pos_err
        elif abs(target_yaw - math.pi) < 0.1 or abs(target_yaw + math.pi) < 0.1: correction -= K_POS * pos_err
        
        cmd.angular.z = max(-0.5, min(0.5, correction))
        self.cmd_pub.publish(cmd)

    def lidar_callback(self, scan):
        if self.front_idx is None: self.compute_indices(scan)
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 12.0)
        self.current_front_dist = ranges[self.front_idx]

        if self.state == "MEASURING":
            if self.current_front_dist < 0.6:
                self.map_length = abs(self.current_x) + self.current_front_dist
                msg = Float32(); msg.data = float(self.map_length); self.dim_pub.publish(msg)
                self.stop_robot(); self.state = "BACKING_AFTER_MEASURE"; return
            right = ranges[self.right_idx]
            cmd = Twist(); error = 0.5 - right; cmd.linear.x = 0.4; cmd.angular.z = 1.5 * error; self.cmd_pub.publish(cmd)

        elif self.state == "BACKING_AFTER_MEASURE":
            if self.move_backward(1.0): self.state = "WAITING"

        elif self.state == "WAITING":
            self.check_start_cleaning()

        elif self.state == "CLEANING":
            if self.clean_step == 0:
                if self.turn_in_place(math.pi/2): self.clean_step = 1
            elif self.clean_step == 1:
                if self.fixed_axis_value == 0.0: self.fixed_axis_value = self.current_x
                if self.current_y < (self.map_width - 0.5) and self.current_front_dist > 0.6: self.move_straight_locked(math.pi/2, 'x', self.fixed_axis_value)
                else: self.fixed_axis_value = 0.0; self.clean_step = 2
            elif self.clean_step == 2:
                if self.move_backward(1.0): self.clean_step = 3
            elif self.clean_step == 3:
                if self.turn_in_place(math.pi): self.clean_step = 4
            elif self.clean_step == 4:
                if self.step_start_pos == 0.0: self.fixed_axis_value = self.current_y; self.step_start_pos = self.current_x
                if self.current_x <= self.partition_min_x: self.stop_robot(); return
                if abs(self.current_x - self.step_start_pos) < 2.0: self.move_straight_locked(math.pi, 'y', self.fixed_axis_value)
                else: self.step_start_pos = 0.0; self.clean_step = 5
            elif self.clean_step == 5:
                if self.wait_in_place(2.0): self.clean_step = 6
            elif self.clean_step == 6:
                if self.turn_in_place(-math.pi/2): self.clean_step = 7
            elif self.clean_step == 7:
                if self.fixed_axis_value == 0.0: self.fixed_axis_value = self.current_x
                if self.current_y > 0.5 and self.current_front_dist > 0.6: self.move_straight_locked(-math.pi/2, 'x', self.fixed_axis_value)
                else: self.fixed_axis_value = 0.0; self.clean_step = 8
            elif self.clean_step == 8:
                if self.move_backward(1.0): self.clean_step = 9
            elif self.clean_step == 9:
                if self.turn_in_place(math.pi): self.clean_step = 10
            elif self.clean_step == 10:
                 if self.step_start_pos == 0.0: self.fixed_axis_value = self.current_y; self.step_start_pos = self.current_x
                 if self.current_x <= self.partition_min_x: self.stop_robot(); return
                 if abs(self.current_x - self.step_start_pos) < 2.0: self.move_straight_locked(math.pi, 'y', self.fixed_axis_value)
                 else: self.step_start_pos = 0.0; self.clean_step = 11
            elif self.clean_step == 11:
                if self.wait_in_place(2.0): self.clean_step = 0

    def check_start_cleaning(self):
        if self.state == "WAITING" and self.map_width is not None and self.map_length is not None:
            self.state = "CLEANING"
            self.partition_min_x = self.map_length / 2.0
            self.get_logger().info(f"R2 Partition: X [{self.partition_min_x:.2f} -> {self.map_length:.2f}]")
        elif self.state == "WAITING":
            pass

    def stop_robot(self): self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(XAxisTrackerAndCleaner())
    rclpy.shutdown()

if __name__ == '__main__': main()