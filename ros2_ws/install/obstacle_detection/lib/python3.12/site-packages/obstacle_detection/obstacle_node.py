import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Subscribe to LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Distance threshold in meters
        self.threshold = 0.5

    def scan_callback(self, msg):
        # Focus on the front sector of the LiDAR
        front_ranges = msg.ranges[len(msg.ranges)//3 : 2*len(msg.ranges)//3]

        # Remove invalid readings (zero or inf)
        valid_ranges = [r for r in front_ranges if r > 0.0]

        if not valid_ranges:
            return

        min_distance = min(valid_ranges)

        if min_distance < self.threshold:
            self.get_logger().info('Obstacle detected')
        else:
            self.get_logger().info('Path clear')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

