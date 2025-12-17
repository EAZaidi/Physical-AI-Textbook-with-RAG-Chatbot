#!/usr/bin/env python3
"""
ROS 2 Node for Collecting Sensor Data
Subscribes to LiDAR, depth camera, and IMU topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu
from std_msgs.msg import Header
import time


class SensorDataCollector(Node):
    def __init__(self):
        super().__init__('sensor_data_collector')

        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/robot/lidar/points',
            self.lidar_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/robot/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10
        )

        # Data counters
        self.lidar_count = 0
        self.depth_count = 0
        self.imu_count = 0

        # Start time
        self.start_time = time.time()

        self.get_logger().info('Sensor Data Collector started')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  /robot/lidar/points')
        self.get_logger().info('  /robot/camera/depth/image_raw')
        self.get_logger().info('  /robot/imu/data')

    def lidar_callback(self, msg):
        self.lidar_count += 1
        if self.lidar_count % 10 == 0:
            self.get_logger().info(f'LiDAR: {self.lidar_count} messages received')

    def depth_callback(self, msg):
        self.depth_count += 1
        if self.depth_count % 30 == 0:
            self.get_logger().info(
                f'Depth Camera: {self.depth_count} messages received '
                f'({msg.width}x{msg.height})'
            )

    def imu_callback(self, msg):
        self.imu_count += 1
        if self.imu_count % 100 == 0:
            elapsed = time.time() - self.start_time
            self.get_logger().info(
                f'IMU: {self.imu_count} messages received '
                f'({self.imu_count/elapsed:.1f} Hz)'
            )


def main(args=None):
    rclpy.init(args=args)
    collector = SensorDataCollector()

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Shutting down...')
        collector.get_logger().info(f'Final counts:')
        collector.get_logger().info(f'  LiDAR: {collector.lidar_count}')
        collector.get_logger().info(f'  Depth: {collector.depth_count}')
        collector.get_logger().info(f'  IMU: {collector.imu_count}')
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
