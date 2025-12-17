#!/usr/bin/env python3
"""
Export ROS 2 IMU data to CSV file
Usage: python3 export_imu_csv.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import time


class IMUExporter(Node):
    def __init__(self):
        super().__init__('imu_exporter')

        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10
        )

        # Open CSV file
        self.csv_file = open('imu_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        self.csv_writer.writerow([
            'timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
            'qx', 'qy', 'qz', 'qw'
        ])

        self.sample_count = 0
        self.max_samples = 1000
        self.start_time = None

        self.get_logger().info(f'Recording {self.max_samples} IMU samples to imu_data.csv...')

    def imu_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()

        # Extract data
        timestamp = time.time() - self.start_time
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Write to CSV
        self.csv_writer.writerow([
            f'{timestamp:.3f}',
            f'{ax:.6f}', f'{ay:.6f}', f'{az:.6f}',
            f'{gx:.6f}', f'{gy:.6f}', f'{gz:.6f}',
            f'{qx:.6f}', f'{qy:.6f}', f'{qz:.6f}', f'{qw:.6f}'
        ])

        self.sample_count += 1

        if self.sample_count % 100 == 0:
            self.get_logger().info(
                f'Recorded {self.sample_count}/{self.max_samples} samples '
                f'({timestamp:.1f}s elapsed)'
            )

        if self.sample_count >= self.max_samples:
            self.finalize()
            raise SystemExit

    def finalize(self):
        self.csv_file.close()
        self.get_logger().info('IMU data saved to imu_data.csv')
        self.get_logger().info(f'Total samples: {self.sample_count}')
        self.get_logger().info('Open with: python3 plot_imu_data.py')

    def __del__(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    exporter = IMUExporter()

    try:
        rclpy.spin(exporter)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        exporter.get_logger().info('Interrupted - finalizing CSV...')
        exporter.finalize()
    finally:
        exporter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
