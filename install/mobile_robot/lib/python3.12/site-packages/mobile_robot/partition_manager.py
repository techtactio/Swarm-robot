import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DecentralizedPartition(Node):
    def __init__(self):
        super().__init__('partition_logic_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 2)

        self.my_id = self.get_parameter('robot_id').value
        self.total_bots = self.get_parameter('total_robots').value

        self.map_length = None
        self.map_width = None
        self.partition_calculated = False

        # Listen for the measurements from trackers
        self.create_subscription(Float32, '/map/length', self.length_callback, 10)
        self.create_subscription(Float32, '/map/width', self.width_callback, 10)

        self.get_logger().info(f"Robot {self.my_id} Partition Node Ready. Waiting for measurements...")

    def length_callback(self, msg):
        if self.map_length is None:
            self.map_length = msg.data
            self.get_logger().info(f"Received Length: {self.map_length:.2f}m")
            self.check_and_partition()

    def width_callback(self, msg):
        if self.map_width is None:
            self.map_width = msg.data
            self.get_logger().info(f"Received Width: {self.map_width:.2f}m")
            self.check_and_partition()

    def check_and_partition(self):
        if self.map_length is not None and self.map_width is not None and not self.partition_calculated:
            self.calculate_my_zone()

    def calculate_my_zone(self):
        self.get_logger().info("Both dimensions received. Calculating partition...")

        strip_width = self.map_length / self.total_bots
        my_index = self.my_id - 1
        
        x_start = my_index * strip_width
        x_end = (my_index + 1) * strip_width
        
        y_start = 0.0
        y_end = self.map_width

        self.my_zone = {
            'x_min': x_start, 'x_max': x_end,
            'y_min': y_start, 'y_max': y_end
        }
        
        self.partition_calculated = True
        
        self.get_logger().info("="*40)
        self.get_logger().info(f"PARTITION ASSIGNED TO ROBOT {self.my_id}")
        self.get_logger().info(f"Zone X: {x_start:.2f}m to {x_end:.2f}m")
        self.get_logger().info(f"Zone Y: {y_start:.2f}m to {y_end:.2f}m")
        self.get_logger().info("Ready for Cleaning...")
        self.get_logger().info("="*40)

def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedPartition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()