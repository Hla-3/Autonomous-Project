import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64
import numpy as np

class LidarObstacle(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.dist_pub = self.create_publisher(Float64, '/obstacle_distance', 10)

        self.threshold = 0.6  # meters

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        front = ranges[len(ranges)//3 : 2*len(ranges)//3]
        min_dist = np.nanmin(front)

        obstacle = min_dist < self.threshold
        self.obstacle_pub.publish(Bool(data=obstacle))
        self.dist_pub.publish(Float64(data=float(min_dist)))

def main():
    rclpy.init()
    node = LidarObstacle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
