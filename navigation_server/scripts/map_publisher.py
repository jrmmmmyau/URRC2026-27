import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image


class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')

        # Load the PGM map
        image = Image.open(
            '/home/wajdar/ros2_ws/URRC2026-27/navigation_server/maps/map2.pgm'
        )

        # Convert the image to grayscale
        image = image.convert('L')

        self.get_logger().info(
            f'Loaded map: {image.width} x {image.height}'
        )

        # Create the OccupancyGrid message
        self.map = OccupancyGrid()

        # Map metadata
        self.map.header.frame_id = 'map'
        self.map.info.resolution = 1.0
        self.map.info.width = image.width
        self.map.info.height = image.height

        # Origin of the map
        self.map.info.origin.position.x = 0.0
        self.map.info.origin.position.y = 0.0
        self.map.info.origin.position.z = 0.0

        self.map.info.origin.orientation.w = 1.0

        # Convert pixels to occupancy values
        self.map.data = []

        pixels = image.load()

        for y in range(image.height):
            for x in range(image.width):

                pixel = pixels[x, image.height - 1 - y]

                if pixel > 127:
                    # White = free
                    self.map.data.append(0)
                else:
                    # Black = obstacle
                    self.map.data.append(100)

        self.get_logger().info(
            f'Created occupancy grid with {len(self.map.data)} cells'
        )

        # Publisher
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        # Publish the map periodically
        self.timer = self.create_timer(
            1.0,
            self.publish_map
        )

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map)


def main(args=None):
    rclpy.init(args=args)

    node = MapPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()