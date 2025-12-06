import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler

topic1 = '/cmd_vel'
topic2 = '/odom1'
topic3 = '/scan'


class ControllerNode(Node):

    def __init__(self, xdu, ydu, kau, kru, kthetau, gstaru, eps_orientu, eps_controlu):
        super().__init__('controller_node')

        self.xdp = xdu
        self.ydp = ydu

        # Tunable parameters (fixed & balanced)
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
        e = thetad_ - theta_
        return math.atan2(math.sin(e), math.cos(e))


    def SensorCallbackPose(self, msg):
        self.OdometryMsg = msg


    def SensorCallbackLidar(self, msg):
        self.LidarMsg = msg


    def ControlFunction(self):

        if self.goal_reached:
            self.controlVel.linear.x = 0.0
            self.controlVel.angular.z = 0.0
            self.controlPublisher.publish(self.controlVel)
            return

        # --- Pose ---
        x = self.OdometryMsg.pose.pose.position.x
        y = self.OdometryMsg.pose.pose.position.y
        q = self.OdometryMsg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        theta = yaw

        # --- Distance to goal ---
        vectorD = np.array([[self.xdp - x], [self.ydp - y]])
        dist = np.linalg.norm(vectorD)

        if dist < self.eps_control:
            self.goal_reached = True
            print(f"Goal reached at ({x:.3f}, {y:.3f})")
            return

        # --- Emergency Stop ---
        if hasattr(self.LidarMsg, 'ranges') and len(self.LidarMsg.ranges) > 0:
            clean_ranges = np.array(self.LidarMsg.ranges)
            clean_ranges = clean_ranges[np.isfinite(clean_ranges)]

            if len(clean_ranges) > 0 and np.min(clean_ranges) < 0.25:
                self.controlVel.linear.x = 0.0
                self.controlVel.angular.z = 0.0
                self.controlPublisher.publish(self.controlVel)
                print("EMERGENCY STOP")
                return


        # --- Attractive Force ---
        AF = self.kap * vectorD

        # --- Repulsive Force ---
        RF = np.zeros((2,1))
        near = False

        if hasattr(self.LidarMsg, 'ranges'):

            ranges = np.array(self.LidarMsg.ranges)
            valid = np.where(np.isfinite(ranges))[0]

            for i in valid:
                r = ranges[i]

                if 0.1 < r < self.gstarp:
                    near = True

                    ang = self.LidarMsg.angle_min + i * self.LidarMsg.angle_increment + theta

                    obs = np.array([[x + r * np.cos(ang)],
                                    [y + r * np.sin(ang)]])
                    diff = np.array([[x - obs[0,0]],
                                     [y - obs[1,0]]])

                    norm = np.linalg.norm(diff)
                    if norm == 0:
                        continue

                    pr = self.krp * ((1/r) - (1/self.gstarp)) / r
                    radial = diff / norm
                    tang = np.array([[-radial[1,0]], [radial[0,0]]])

                    RF += pr * (radial + 0.6 * tang)

        # Reduce attraction when obstacles nearby
        if near:
            AF *= 0.4


        # --- Wall Repulsion ---
        L = 10.0
        g = 1.5
        k = 6.0
        gradUw = np.zeros((2,1))

        if x < -L + g:
            gradUw += np.array([[k], [0]])
        if x > L - g:
            gradUw += np.array([[-k], [0]])
        if y < -L + g:
            gradUw += np.array([[0], [k]])
        if y > L - g:
            gradUw += np.array([[0], [-k]])

        if dist < 1.0:
            gradUw *= 0.0


        # --- Total Force ---
        F = AF + RF + gradUw
        # Deadlock escape boost
        if near and np.linalg.norm(RF) > np.linalg.norm(AF):
            F += 0.3 * np.array([[-np.sin(theta)], [np.cos(theta)]])


        # --- Normalize force ---
        maxF = 2.0
        normF = np.linalg.norm(F)
        if normF > maxF:
            F = (F / normF) * maxF


        # --- Heading ---
        thetaD = math.atan2(F[1,0], F[0,0])
        e = self.orientationError(theta, thetaD)


        # --- Velocity Control ---
        if abs(e) > self.eps_orient:
            v = 0.0
            w = self.kthetap * e
        else:
            v = np.linalg.norm(F)
            w = self.kthetap * e

            # Smooth slowdown near goal
            if dist < 1.0:
                v = 0.5 + 0.5 * dist



        # --- Publish ---
        self.controlVel.linear.x = float(v)
        self.controlVel.angular.z = float(w)
        self.controlPublisher.publish(self.controlVel)

        print(f"x={x:.2f} y={y:.2f} θ={theta:.2f} dist={dist:.2f} v={v:.2f} ω={w:.2f}")



def main(args=None):

    rclpy.init(args=args)

    # --- Better TUNING ---
    xd = 11
    yd = 11

    kap = 0.25
    krp = 1.5
    ktheta = 2.5
    gstarp = 2.0
    eps_orient = math.pi / 20
    eps_control = 0.05

    node = ControllerNode(xd, yd, kap, krp, ktheta, gstarp, eps_orient, eps_control)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
