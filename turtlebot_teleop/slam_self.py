#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import numpy as np
import math


def bresenham_line(x0, y0, x1, y1):
    """Returns a list of grid cells from (x0, y0) to (x1, y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    points = []
    
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')
        # Parameters for the map
        self.resolution = 0.05  # meters per cell
        self.width = 400        # cells in x
        self.height = 400       # cells in y
        # Map origin in the world (lower left corner of the grid)
        self.origin = [-self.width/2 * self.resolution, -self.height/2 * self.resolution, 0.0]

        # Occupancy grid: -1 unknown, 0 free, 100 occupied
        self.map = np.full((self.height, self.width), -1, dtype=np.int8)

        # Publishers and Subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.robot_pose = [0.0, 0.0, 0.0]

    def odom_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_pose[2] = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Compute the hit point in the world frame using current robot pose
                hit_x = self.robot_pose[0] + r * math.cos(self.robot_pose[2] + angle)
                hit_y = self.robot_pose[1] + r * math.sin(self.robot_pose[2] + angle)

                # Convert world coordinates to map indices for the hit cell
                mx = int((hit_x - self.origin[0]) / self.resolution)
                my = int((hit_y - self.origin[1]) / self.resolution)
                
                # Convert the robot's current position to map indices for raycasting
                robot_mx = int((self.robot_pose[0] - self.origin[0]) / self.resolution)
                robot_my = int((self.robot_pose[1] - self.origin[1]) / self.resolution)
                
                line_points = bresenham_line(robot_mx, robot_my, mx, my)
                for (cell_x, cell_y) in line_points[:-1]:
                    if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
                        self.map[cell_y, cell_x] = 0  # Mark as free

                if 0 <= mx < self.width and 0 <= my < self.height:
                    self.map[my, mx] = 100  # Mark as occupied

            angle += msg.angle_increment

        self.publish_map()

    def publish_map(self):
        # Create and publish an OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        meta = MapMetaData()
        meta.resolution = self.resolution
        meta.width = self.width
        meta.height = self.height
        meta.origin.position.x = self.origin[0]
        meta.origin.position.y = self.origin[1]
        grid.info = meta

        grid.data = self.map.flatten().tolist()
        self.map_pub.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
