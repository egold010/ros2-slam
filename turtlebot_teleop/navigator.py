#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.action
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        # OccupancyGrid map and Odometry for current robot pose.
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Action client for NavigateToPose
        self.nav_action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer to periodically run frontier detection and exploration
        self.timer = self.create_timer(0.5, self.explore_callback)
        
        self.robot_pose = [0.0, 0.0, 0.0]
        self.map_data = None
        self.map_info = None
        
        self.goal_active = False

    def odom_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_pose[2] = math.atan2(siny, cosy)

    def map_callback(self, msg):
        # Save the map metadata and convert the data to a 2D numpy array.
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def detect_frontier(self):
        if self.map_data is None or self.map_info is None:
            return None

        resolution = self.map_info.resolution
        width = self.map_info.width
        height = self.map_info.height
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        best_distance = float('inf')
        best_frontier = None

        # Iterate through the grid
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] == 0:  # free cell
                    # Check 4-connected neighbors for unknown cells
                    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
                    for nx, ny in neighbors:
                        if 0 <= nx < width and 0 <= ny < height:
                            if self.map_data[ny, nx] == -1: # Frontier found, convert cell indices to world coordinates
                                world_x = origin_x + (x + 0.5) * resolution
                                world_y = origin_y + (y + 0.5) * resolution
                                dx = world_x - self.robot_pose[0]
                                dy = world_y - self.robot_pose[1]
                                distance = math.hypot(dx, dy)
                                if 2 < distance < best_distance:
                                    best_distance = distance
                                    best_frontier = (world_x, world_y)
                                break

        if best_frontier:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = best_frontier[0]
            goal_pose.pose.position.y = best_frontier[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            return goal_pose
        return None

    def explore_callback(self):
        if self.goal_active:
            return  # Skip if a navigation goal is already active

        frontier_goal = self.detect_frontier()
        if frontier_goal is not None:
            self.get_logger().info(
                f"Frontier found at ({frontier_goal.pose.position.x:.2f}, "
                f"{frontier_goal.pose.position.y:.2f})."
            )

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = frontier_goal
            self.goal_active = True
            send_goal_future = self.nav_action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("No frontier found.")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected.")
            self.goal_active = False
            return

        self.get_logger().info("Navigation goal accepted. Waiting for result...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # Here you could inspect result for additional info; we'll assume success.
        self.get_logger().info("Navigation goal reached.")
        self.goal_active = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Navigation feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
