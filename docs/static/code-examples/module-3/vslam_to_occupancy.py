#!/usr/bin/env python3
"""
Convert VSLAM 3D Point Cloud to 2D Occupancy Grid
Module 3, Chapter 3 - For Nav2 planning
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class VslamToOccupancy(Node):
    def __init__(self):
        super().__init__('vslam_to_occupancy')

        # Parameters
        self.declare_parameter('resolution', 0.05)  # 5cm cells
        self.declare_parameter('height_min', 0.0)   # Min Z to consider (ground level)
        self.declare_parameter('height_max', 2.0)   # Max Z to consider (ceiling)

        self.resolution = self.get_parameter('resolution').value
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value

        # Subscriber: VSLAM map points
        self.subscription = self.create_subscription(
            PointCloud2,
            '/visual_slam/vis/map_points',
            self.pointcloud_callback,
            10
        )

        # Publisher: Occupancy grid for Nav2
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.get_logger().info(f'VSLAM to Occupancy converter started')
        self.get_logger().info(f'  Resolution: {self.resolution}m')
        self.get_logger().info(f'  Height range: {self.height_min}m - {self.height_max}m')

    def pointcloud_callback(self, msg):
        try:
            # Extract XYZ points from PointCloud2
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                # Filter by height (only obstacles at humanoid height)
                if self.height_min <= p[2] <= self.height_max:
                    points.append([p[0], p[1]])

            if len(points) < 10:
                self.get_logger().warn('Not enough points for grid generation')
                return

            points = np.array(points)

            # Compute grid bounds
            origin_x = points[:, 0].min() - 1.0  # 1m padding
            origin_y = points[:, 1].min() - 1.0
            max_x = points[:, 0].max() + 1.0
            max_y = points[:, 1].max() + 1.0

            width = int((max_x - origin_x) / self.resolution)
            height = int((max_y - origin_y) / self.resolution)

            # Initialize grid (-1 = unknown, 0 = free, 100 = occupied)
            grid = np.full((height, width), -1, dtype=np.int8)

            # Mark occupied cells
            for p in points:
                x_idx = int((p[0] - origin_x) / self.resolution)
                y_idx = int((p[1] - origin_y) / self.resolution)
                if 0 <= x_idx < width and 0 <= y_idx < height:
                    grid[y_idx, x_idx] = 100  # Occupied

            # Inflate obstacles (simple 1-cell dilation)
            inflated_grid = grid.copy()
            for y in range(1, height - 1):
                for x in range(1, width - 1):
                    if grid[y, x] == 100:
                        # Mark neighbors as occupied
                        inflated_grid[y-1:y+2, x-1:x+2] = 100

            # Create OccupancyGrid message
            occupancy_msg = OccupancyGrid()
            occupancy_msg.header = msg.header
            occupancy_msg.header.frame_id = 'map'
            occupancy_msg.info.resolution = float(self.resolution)
            occupancy_msg.info.width = width
            occupancy_msg.info.height = height
            occupancy_msg.info.origin.position.x = float(origin_x)
            occupancy_msg.info.origin.position.y = float(origin_y)
            occupancy_msg.info.origin.position.z = 0.0
            occupancy_msg.data = inflated_grid.flatten().tolist()

            self.publisher.publish(occupancy_msg)

            self.get_logger().info(f'Published occupancy grid: {width}x{height} cells, {len(points)} points')

        except Exception as e:
            self.get_logger().error(f'Error converting pointcloud: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = VslamToOccupancy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
