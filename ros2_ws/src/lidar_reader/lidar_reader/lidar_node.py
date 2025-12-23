import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader_node')

        # Subscribe to the LiDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # Get the minimum distance from the LiDAR ranges
        min_distance = min(msg.ranges)

        # Print the distance to the closest obstacle
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f} meters')


def main(args=None):
    rclpy.init(args=args)

    node = LidarReader()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

