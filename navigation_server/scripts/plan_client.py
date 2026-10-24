import sys

import rclpy
from rclpy.node import Node
from navigation_server.srv import PlanPath

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D, PoseStamped


class PlanClient(Node):

    def __init__(self, start_x, start_y, goal_x, goal_y):
        super().__init__('plan_client')

        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.map = None
        self.request_sent = False

        # Subscribe to the map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Service client
        self.client = self.create_client(
            PlanPath,
            '/plan_path'
        )

        # Publish the resulting path
        self.path_publisher = self.create_publisher(
            Path,
            '/plan',
            10
        )

        # Publish start and goal
        self.start_publisher = self.create_publisher(
            PoseStamped,
            '/start',
            10
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.send_request
        )

    def map_callback(self, msg):
        self.map = msg

    def send_request(self):

        if self.request_sent:
            return

        if self.map is None:
            self.get_logger().info('Waiting for map...')
            return

        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info('Waiting for /plan_path...')
            return

        # Create the service request
        request = PlanPath.Request()

        # Start
        request.start = Pose2D()
        request.start.x = self.start_x
        request.start.y = self.start_y
        request.start.theta = 0.0

        # Goal
        request.goal = Pose2D()
        request.goal.x = self.goal_x
        request.goal.y = self.goal_y
        request.goal.theta = 0.0

        # Send the map directly to A*
        request.map = self.map

        self.get_logger().info(
            f'Start: ({self.start_x}, {self.start_y})'
        )

        self.get_logger().info(
            f'Goal: ({self.goal_x}, {self.goal_y})'
        )

        self.get_logger().info('Sending map to A*...')

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

        self.request_sent = True

    def response_callback(self, future):

        try:
            response = future.result()

            number_of_poses = len(response.plan.poses)

            self.get_logger().info(
                f'Received path with {number_of_poses} poses'
            )

            if number_of_poses == 0:
                self.get_logger().warn(
                    'A* did not find a path.'
                )
                return

            # Publish path
            self.path_publisher.publish(response.plan)

            # Publish start
            start = PoseStamped()
            start.header = response.plan.header
            start.pose.position.x = self.start_x
            start.pose.position.y = self.start_y
            start.pose.orientation.w = 1.0

            self.start_publisher.publish(start)

            # Publish goal
            goal = PoseStamped()
            goal.header = response.plan.header
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.orientation.w = 1.0

            self.goal_publisher.publish(goal)

            self.get_logger().info(
                'Published path, start, and goal.'
            )

        except Exception as e:
            self.get_logger().error(
                f'Service call failed: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    # Require four coordinates
    if len(sys.argv) != 5:
        print(
            'Usage: python3 plan_client.py '
            'START_X START_Y GOAL_X GOAL_Y'
        )
        return

    start_x = float(sys.argv[1])
    start_y = float(sys.argv[2])
    goal_x = float(sys.argv[3])
    goal_y = float(sys.argv[4])

    node = PlanClient(
        start_x,
        start_y,
        goal_x,
        goal_y
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()