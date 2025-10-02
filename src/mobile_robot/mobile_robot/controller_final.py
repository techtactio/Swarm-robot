import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
import time

topic1 = '/cmd_vel'
topic2 = '/odom1'
topic3 = '/scan'

class ControllerNode(Node):
    def __init__(self, xdu, ydu, kau, kru, kthetau, gstaru, eps_orientu, eps_controlu):
        super().__init__('controller_node')
        self.xdp = xdu
        self.ydp = ydu

        self.kap = kau
        self.krp = kru
        self.kthetap = kthetau
        self.gstarp = gstaru
        self.eps_orient = eps_orientu
        self.eps_control = eps_controlu

        self.OdometryMsg = Odometry()
        self.LidarMsg = LaserScan()

        self.controlVel = Twist()
        self.controlPublisher = self.create_publisher(Twist, topic1, 10)
        self.poseSubscriber = self.create_subscription(Odometry, topic2, self.SensorCallbackPose, 10)
        self.lidarSubscriber = self.create_subscription(LaserScan, topic3, self.SensorCallbackLidar, 10)

        self.period = 0.05
        self.timer = self.create_timer(self.period, self.ControlFunction)
        self.goal_reached = False

    def orientationError(self, theta_, thetad_):
        if (thetad_ > np.pi/2) and (thetad_ <= np.pi):
            if (theta_ > -np.pi) and (theta_ <= -np.pi/2):
                theta_ += 2*np.pi
        if (theta_ > np.pi/2) and (theta_ <= np.pi):
            if (thetad_ > -np.pi) and (thetad_ <= -np.pi/2):
                thetad_ += 2*np.pi
        return thetad_ - theta_

    def SensorCallbackPose(self, receivedMsg):
        self.OdometryMsg = receivedMsg

    def SensorCallbackLidar(self, receivedMsg):
        self.LidarMsg = receivedMsg

    def ControlFunction(self):
        if self.goal_reached:
            self.controlVel.linear.x = 0.0
            self.controlVel.angular.z = 0.0
            self.controlPublisher.publish(self.controlVel)
            return

        x = self.OdometryMsg.pose.pose.position.x
        y = self.OdometryMsg.pose.pose.position.y
        quat = self.OdometryMsg.pose.pose.orientation
        quatl = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = quat2euler([quatl[3], quatl[0], quatl[1], quatl[2]])
        theta = yaw

        # Distance and vector to goal
        vectorD = np.array([[self.xdp - x], [self.ydp - y]])
        dist_to_goal = np.linalg.norm(vectorD)

        # Stop at goal
        if dist_to_goal < self.eps_control:
            print(f"Goal reached at ({x:.3f}, {y:.3f})!")
            self.goal_reached = True
            self.controlVel.linear.x = 0.0
            self.controlVel.angular.z = 0.0
            self.controlPublisher.publish(self.controlVel)
            return

        # Attractive Force (towards goal)
        AF = self.kap * vectorD

        # Obstacle Repulsion
        LidarRanges = np.array(self.LidarMsg.ranges)
        indices = np.where(~np.isinf(LidarRanges))[0]
        RF = np.array([[0.0], [0.0]])
        if indices.size > 0:
            angle_min = self.LidarMsg.angle_min
            angle_inc = self.LidarMsg.angle_increment
            for i in indices:
                r = LidarRanges[i]
                if r < self.gstarp and r > 0.01:  # avoid divide by zero
                    angle = angle_min + i * angle_inc + theta
                    obs_vec = np.array([[x + r*np.cos(angle)], [y + r*np.sin(angle)]])
                    diff = np.array([[x - obs_vec[0,0]], [y - obs_vec[1,0]]])
                    pr = self.krp * (1/self.gstarp - 1/r) * (1/(r**3))
                    RF += pr * diff

        # Wall Repulsion (disabled near goal)
        L = 10.0
        gradUw = np.array([[0.0], [0.0]])
        gstar_wall = 1.5
        kr_wall = 5.0

        # Apply wall repulsion only if robot is not very close to goal
        wall_multiplier = 1.0
        if dist_to_goal < 1.0:
            wall_multiplier = 0.0

        # X walls
        if x <= -L + gstar_wall:
            gradUw += kr_wall * np.array([[1.0], [0.0]])
        if x >= L - gstar_wall:
            gradUw += kr_wall * np.array([[-1.0], [0.0]])
        # Y walls
        if y <= -L + gstar_wall:
            gradUw += kr_wall * np.array([[0.0], [1.0]])
        if y >= L - gstar_wall:
            gradUw += kr_wall * np.array([[0.0], [-1.0]])

        gradUw *= wall_multiplier

        # Total force
        F = AF + RF + gradUw

        # Desired orientation
        thetaD = math.atan2(F[1,0], F[0,0])
        eorient = self.orientationError(theta, thetaD)

        # Velocity Control
        if np.abs(eorient) > self.eps_orient:
            thetavel = self.kthetap * eorient
            xvel = 0.0
        else:
            thetavel = self.kthetap * eorient
            xvel = np.linalg.norm(F)
            # Slow down as approaching goal
            if dist_to_goal < 1.0:
                xvel *= dist_to_goal / 1.0
                xvel = max(xvel, 0.5)

        # Publish velocity
        self.controlVel.linear.x = xvel
        self.controlVel.angular.z = thetavel
        self.controlPublisher.publish(self.controlVel)

        # Debug print
        print(f"x={x:.3f}, y={y:.3f}, theta={theta:.3f}, xvel={xvel:.3f}, thetavel={thetavel:.3f}")

def main(args=None):
    rclpy.init(args=args)
    xd_u = 0
    yd_u = 0
    node = ControllerNode(xd_u, yd_u, 0.6, 0.8, 2.0, 1.0, np.pi/18, 0.05)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
