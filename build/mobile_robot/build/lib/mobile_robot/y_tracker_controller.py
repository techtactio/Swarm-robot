import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class YAxisTracker(Node):
    def __init__(self):
        super().__init__('y_axis_tracker')

        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.lidar_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/robot1/odom1', self.odom_callback, qos_profile_sensor_data)

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.done = False
        self.front_idx = None
        self.left_idx = None
        self.last_log_time = self.get_clock().now()

        self.get_logger().info("Y-Axis Tracker (Robot 1) Ready.")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = raw_x
            self.start_y = raw_y
            self.start_yaw = yaw
            self.get_logger().info(f"R1 Start Pos: {raw_x:.2f}, {raw_y:.2f}")

        dx = raw_x - self.start_x
        dy = raw_y - self.start_y

        self.current_y = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        self.current_x = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)

    def compute_indices(self, scan):
        angle_min = scan.angle_min
        inc = scan.angle_increment
        def a2i(a): return int((a - angle_min) / inc)
        N = len(scan.ranges) - 1
        self.front_idx = max(0, min(N, a2i(0.0)))
        self.left_idx = max(0, min(N, a2i(math.pi/2)))

    def lidar_callback(self, scan):
        if self.done: return
        if self.front_idx is None: self.compute_indices(scan)

        cmd = Twist()
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 12.0)

        front = ranges[self.front_idx]
        left = ranges[self.left_idx]

        # 1. Log Output (Throttled)
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 2e9:
            self.get_logger().info(f"R1 Moving... Y: {abs(self.current_y):.2f}")
            self.last_log_time = now

        # 2. Stop at End & Print Final Coordinates
        if front < 0.6:
            final_y = abs(self.current_y + front)
            final_x = abs(self.current_x)
            
            # FINAL OUTPUT: (Drift, Length) - All Positive
            self.get_logger().info(f"FINAL COORDINATES (Robot 1): ({final_x:.2f}, {final_y:.2f})")
            
            cmd.linear.x = 0.0
            self.done = True
            self.cmd_pub.publish(cmd)
            return

        # 3. Wall Follow (Left)
        desired = 0.5
        if left > desired + 0.1:
            cmd.linear.x = 0.3; cmd.angular.z = 0.3 
        elif left < desired - 0.1:
            cmd.linear.x = 0.3; cmd.angular.z = -0.3 
        else:
            cmd.linear.x = 0.9; cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = YAxisTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()