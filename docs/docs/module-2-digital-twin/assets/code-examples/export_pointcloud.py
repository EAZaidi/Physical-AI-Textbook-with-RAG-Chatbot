#!/usr/bin/env python3
"""
Export ROS 2 PointCloud2 to PCD file
Usage: python3 export_pointcloud.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np


class PointCloudExporter(Node):
    def __init__(self):
        super().__init__('pointcloud_exporter')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/robot/lidar/points',
            self.pointcloud_callback,
            10
        )

        self.scan_count = 0
        self.max_scans = 100
        self.points_list = []

        self.get_logger().info(f'Collecting {self.max_scans} point cloud scans...')

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 to list of points
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_array = np.array([[p[0], p[1], p[2]] for p in points])

        self.points_list.extend(points_array)
        self.scan_count += 1

        if self.scan_count % 10 == 0:
            self.get_logger().info(f'Collected {self.scan_count}/{self.max_scans} scans')

        if self.scan_count >= self.max_scans:
            self.export_pcd()
            raise SystemExit

    def export_pcd(self):
        self.get_logger().info(f'Exporting {len(self.points_list)} points to PCD...')

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(self.points_list))

        # Save to file
        output_file = 'output.pcd'
        o3d.io.write_point_cloud(output_file, pcd)

        self.get_logger().info(f'Point cloud saved to {output_file}')
        self.get_logger().info(f'Total points: {len(self.points_list)}')
        self.get_logger().info(f'Open with: cloudcompare {output_file}')


def main(args=None):
    rclpy.init(args=args)
    exporter = PointCloudExporter()

    try:
        rclpy.spin(exporter)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        exporter.get_logger().info('Interrupted - exporting partial data...')
        if exporter.points_list:
            exporter.export_pcd()
    finally:
        exporter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
