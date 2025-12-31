#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarFixer(Node):
    def __init__(self):
        super().__init__('lidar_fixer')
        # 전방 라이다 구독/발행
        self.front_sub = self.create_subscription(LaserScan, '/scan_front_raw', self.front_callback, 10)
        self.front_pub = self.create_publisher(LaserScan, '/scan_front_fixed', 10)

        # 후방 라이다 구독/발행
        self.rear_sub = self.create_subscription(LaserScan, '/scan_rear_raw', self.rear_callback, 10)
        self.rear_pub = self.create_publisher(LaserScan, '/scan_rear_fixed', 10)

    def front_callback(self, msg):
        fixed_msg = self.fix_timestamp(msg)
        self.front_pub.publish(fixed_msg)

    def rear_callback(self, msg):
        fixed_msg = self.fix_timestamp(msg)
        self.rear_pub.publish(fixed_msg)

    def fix_timestamp(self, msg):
        # Cartographer 에러 방지를 위한 시간 값 주입 
        if msg.time_increment == 0.0:
            msg.time_increment = 0.00001
        if msg.scan_time == 0.0:
            msg.scan_time = 0.01
        return msg


def main():
    rclpy.init()
    node = LidarFixer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
