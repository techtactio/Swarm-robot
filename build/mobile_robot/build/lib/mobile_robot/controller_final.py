import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
from transforms3d.euler import quat2euler


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom1', self.odom_callback, 10)

        # Spawn-frame storage
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # Current local-frame position
        self.X = 0.0
        self.Y = 0.0
        self.yaw = None

        self.turn_start_yaw = None
        self.turning_90 = False
        self.done = False

        self.front_idx = None
        self.left_idx = None
        self.right_idx = None

        self.get_logger().info("Wall follower with local coordinate frame active.")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        self.yaw = yaw

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        # Save spawn point and heading
        if self.start_x is None:
            self.start_x = raw_x
            self.start_y = raw_y
            self.start_yaw = yaw
            self.get_logger().info("Local frame initialized at spawn.")

        dx = raw_x - self.start_x
        dy = raw_y - self.start_y

        # Project movement onto robot's forward (+Y) axis
        self.Y = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)

        # Sideways drift (not used but available)
        self.X = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)


    def compute_indices(self, scan):
        angle_min = scan.angle_min
        inc = scan.angle_increment

        def a2i(a):
            return int((a - angle_min) / inc)

        self.front_idx = a2i(0.0)
        self.left_idx  = a2i(math.pi/2)
        self.right_idx = a2i(-math.pi/2)

        N = len(scan.ranges)
        self.front_idx = max(0, min(N-1, self.front_idx))
        self.left_idx  = max(0, min(N-1, self.left_idx))
        self.right_idx = max(0, min(N-1, self.right_idx))

        self.get_logger().info(f"Indices: F={self.front_idx}, L={self.left_idx}, R={self.right_idx}")

    def lidar_callback(self, scan):
        if self.done:
            return

        if self.front_idx is None:
            self.compute_indices(scan)

        cmd = Twist()

        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 12.0)

        def w(idx):
            return np.min(ranges[max(0, idx - 10): min(len(ranges), idx + 10)])

        front = w(self.front_idx)
        left = w(self.left_idx)
        right = w(self.right_idx)

        # Debug prints
        self.get_logger().info(
            f"front={front:.2f} left={left:.2f} right={right:.2f}  |  LocalPos X={self.X:.3f}, Y={self.Y:.3f}"
        )

        # --------------------------------------------------------
        # Turning 90 degrees
        # --------------------------------------------------------
        if self.turning_90:
            cmd.angular.z = -1.0  # rotate CW

            if self.yaw is not None:
                angle_turned = abs(self.normalize(self.yaw - self.turn_start_yaw))
                if angle_turned >= math.pi/2:
                    self.get_logger().info("90-degree turn complete.")
                    cmd.angular.z = 0.0
                    self.done = True
                    self.cmd_pub.publish(cmd)
                    return

            self.cmd_pub.publish(cmd)
            return

        # --------------------------------------------------------
        # Detect corner → begin 90-degree turn
        # --------------------------------------------------------
        if front < 0.6:
            # Print clean coordinate: X should remain ~0, Y is forward distance
            self.get_logger().info(
                f"Backing start point → X={self.X:.3f}, Y={self.Y:.3f}"
            )

            self.get_logger().info("Corner detected. Starting 90-degree turn.")
            self.turning_90 = True
            self.turn_start_yaw = self.yaw
            return

        # --------------------------------------------------------
        # Normal wall-following movement
        # --------------------------------------------------------
        desired = 0.5

        if left > desired + 0.1:
            cmd.linear.x = 0.3
            cmd.angular.z = +0.5
        elif left < desired - 0.1:
            cmd.linear.x = 0.3
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def normalize(self, a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
