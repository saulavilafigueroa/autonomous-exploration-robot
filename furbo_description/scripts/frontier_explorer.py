#!/usr/bin/env python3

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import numpy as np
from collections import deque


def is_frontier_cell(grid, x, y, width, height):
    if grid[y][x] != 0:
        return False

    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

    for nx, ny in neighbors:
        if 0 <= nx < width and 0 <= ny < height:
            if grid[ny][nx] == -1:
                return True

    return False


def cluster_frontier_cells(frontier_cells, width, height):
    frontier_set = set(frontier_cells)
    visited = set()
    clusters = []

    for cell in frontier_cells:
        if cell in visited:
            continue

        cluster = []
        queue = deque([cell])
        visited.add(cell)

        while queue:
            current = queue.popleft()
            cluster.append(current)

            x, y = current
            neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

            for neighbor in neighbors:
                if neighbor in frontier_set and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        clusters.append(cluster)

    return clusters


def select_nearest_frontier(clusters, resolution, origin_x, origin_y, robot_x, robot_y, failed_goals, min_distance=1.0, avoid_radius=1.0):
    best_point = None
    best_distance = float('inf')

    for cluster in clusters:
        if len(cluster) < 3:
            continue

        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)

        world_x = origin_x + cx * resolution
        world_y = origin_y + cy * resolution

        distance = ((world_x - robot_x) ** 2 + (world_y - robot_y) ** 2) ** 0.5

        if distance < min_distance:
            continue

        too_close_to_failure = False
        for fx, fy in failed_goals:
            fail_distance = ((world_x - fx) ** 2 + (world_y - fy) ** 2) ** 0.5
            if fail_distance < avoid_radius:
                too_close_to_failure = True
                break

        if too_close_to_failure:
            continue

        if distance < best_distance:
            best_distance = distance
            best_point = (world_x, world_y)

    return best_point


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.map_data = None
        self.exploring = False

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(2.0, self.explore_step)
        self.failed_goals = []
        self.current_goal = None

    def map_callback(self, msg):
        self.map_data = msg

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f'Could not get robot position: {e}')
            return None

    def explore_step(self):
        if self.exploring:
            return

        if self.map_data is None:
            self.get_logger().info('Waiting for first map...')
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y = robot_pose

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        grid = np.array(self.map_data.data).reshape((height, width))

        frontier_cells = []
        for y in range(height):
            for x in range(width):
                if is_frontier_cell(grid, x, y, width, height):
                    frontier_cells.append((x, y))

        if not frontier_cells:
            self.get_logger().info('No frontiers left -- exploration complete.')
            self.timer.cancel()
            return

        clusters = cluster_frontier_cells(frontier_cells, width, height)
        target = select_nearest_frontier(
            clusters, resolution, origin_x, origin_y, robot_x, robot_y, self.failed_goals
        )

        if target is None:
            self.get_logger().info('No valid frontiers left -- exploration complete.')
            self.timer.cancel()
            return

        self.send_goal(target)

    def send_goal(self, target):
        goal_x, goal_y = target
        self.current_goal = target
        self.get_logger().info(f'Sending goal to: ({goal_x:.2f}, {goal_y:.2f})')

        self.nav_client.wait_for_server()
        self.exploring = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.exploring = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal SUCCEEDED, looking for next frontier...')
        else:
            self.get_logger().warn(f'Goal did NOT succeed (status={status}), blacklisting this area...')
            self.failed_goals.append(self.current_goal)
        self.exploring = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()