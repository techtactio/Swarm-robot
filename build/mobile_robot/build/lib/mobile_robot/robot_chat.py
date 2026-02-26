import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import json

class RobotChat(Node):
    def __init__(self):
        super().__init__('robot_chat')
        
        self.my_name = self.get_namespace().strip('/')
        self.known_robots = {}

        # 1. THE FIX: Listen to the ACTUAL tracker telemetry, not a separate chat!
        self.telemetry_sub = self.create_subscription(String, '/swarm/telemetry', self.telemetry_callback, 10)
        
        self.count_pub = self.create_publisher(Int32, '/swarm_count', 10)

        # 2. Faster timeout: Since telemetry runs at 10Hz, 2 seconds of silence means it crashed.
        self.health_timer = self.create_timer(1.0, self.check_active_robots)
        self.DEAD_TIMEOUT_NS = 2.0 * 1e9 

        self.get_logger().info(f"Swarm Health Monitor initialized for: {self.my_name}")

    def telemetry_callback(self, msg):
        try:
            data = json.loads(msg.data)
            sender_name = data["sender"]
            now = self.get_clock().now().nanoseconds
            
            # If we see a new tracker online
            if sender_name not in self.known_robots:
                self.known_robots[sender_name] = now
                count = len(self.known_robots)
                
                self.get_logger().info(f'👋 Found active robot: {sender_name}. Total: {count}')
                
                count_msg = Int32()
                count_msg.data = count
                self.count_pub.publish(count_msg)
            else:
                # Keep the heartbeat fresh
                self.known_robots[sender_name] = now
        except Exception as e:
            pass

    def check_active_robots(self):
        now = self.get_clock().now().nanoseconds
        dead_robots = []
        
        # Check if ANY robot's tracker has stopped publishing telemetry
        for robot, last_seen in self.known_robots.items():
            if (now - last_seen) > self.DEAD_TIMEOUT_NS:
                dead_robots.append(robot)
                
        # If a tracker died, remove it and broadcast the lower count
        if dead_robots:
            for dead in dead_robots:
                del self.known_robots[dead]
                self.get_logger().warn(f'💀 Robot tracker lost: {dead}. Total now: {len(self.known_robots)}')
            
            count_msg = Int32()
            count_msg.data = len(self.known_robots)
            self.count_pub.publish(count_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotChat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
