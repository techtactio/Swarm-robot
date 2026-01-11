import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32  # Import Int32

class RobotChat(Node):
    def __init__(self):
        super().__init__('robot_chat')
        
        self.my_name = self.get_namespace().strip('/')
        self.known_robots = set()
        self.known_robots.add(self.my_name)

        # 1. Swarm Chat (Existing)
        self.chat_pub = self.create_publisher(String, '/swarm_chat', 10)
        self.chat_sub = self.create_subscription(String, '/swarm_chat', self.listener_callback, 10)
        
        # 2. COUNT PUBLISHER (New!)
        # We publish the count globally so everyone agrees on the number
        self.count_pub = self.create_publisher(Int32, '/swarm_count', 10)

        self.timer = self.create_timer(2.0, self.say_hello)
        self.get_logger().info(f"Robot Chat initialized for: {self.my_name}")

    def say_hello(self):
        msg = String()
        msg.data = self.my_name
        self.chat_pub.publish(msg)
        
        # Also re-publish the current count periodically, just in case
        count_msg = Int32()
        count_msg.data = len(self.known_robots)
        self.count_pub.publish(count_msg)

    def listener_callback(self, msg):
        sender_name = msg.data
        
        if sender_name not in self.known_robots:
            self.known_robots.add(sender_name)
            count = len(self.known_robots)
            
            self.get_logger().info(f'👋 Found new robot: {sender_name}. Total: {count}')
            
            # Publish new count immediately when found
            count_msg = Int32()
            count_msg.data = count
            self.count_pub.publish(count_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotChat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()