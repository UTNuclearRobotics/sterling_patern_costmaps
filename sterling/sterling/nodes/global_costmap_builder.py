import os
from datetime import datetime

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

# Define a QoS profile with Transient Local durability
qos_profile = QoSProfile(
    depth=10,  # Queue size
    history=QoSHistoryPolicy.KEEP_LAST,  # Keep last N messages
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Transient Local durability
)


class GlobalCostmapBuilder(Node):
    """
    Listens to the global costmap know origin and when to resize the map.
    The only values being placed onto the global costamp are stitched sterling local costmaps.

    Requires local_costmap, global_costmap, and slam_toolbox map resolutions to be the same.
    """

    def __init__(self):
        super().__init__("global_costmap_builder")

        # Declare and get parameters
        self.declare_parameter("local_costmap_topic", "/panther/local_costmap/costmap")
        self.declare_parameter("global_costmap_topic", "/panther/global_costmap/costmap")
        self.declare_parameter("use_maximum", False)

        self.local_costmap_topic = self.get_parameter("local_costmap_topic").value
        self.global_costmap_topic = self.get_parameter("global_costmap_topic").value
        self.use_maximum = self.get_parameter("use_maximum").value

        # Print parameter values
        self.get_logger().debug(f"Local costmap topic: {self.local_costmap_topic}")
        self.get_logger().debug(f"Global costmap topic: {self.global_costmap_topic}")
        self.get_logger().debug(f"Use maximum: {self.use_maximum}")

        # Subscribe to the local costmap
        self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self.stitch_local_publish_global,
            10,
        )

        # Subscribe to the global costmap
        self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            self.global_costmap_callback,
            10,
        )

        # Publisher for the global costmap
        self.stitched_costmap_publisher = self.create_publisher(
            OccupancyGrid,
            "global_costmap",
            qos_profile=qos_profile,
        )

        # Create a service to save the costmap
        self.service = self.create_service(Trigger, "save_costmap", self.save_costmap_callback)

        # Initialize stitched costmap
        self.stitched_costmap = None
        self.update_msg = None

    def global_costmap_callback(self, msg):
        """Callback for the global costmap."""
        self.global_msg = msg
        self.global_resolution = msg.info.resolution
        self.global_width = msg.info.width
        self.global_height = msg.info.height
        self.global_origin_x = msg.info.origin.position.x
        self.global_origin_y = msg.info.origin.position.y

        # Initialize stitched costmap if not yet initialized
        if self.stitched_costmap is None:
            self.stitched_costmap = np.full((self.global_height, self.global_width), -1, dtype=int)
            self.stitched_resolution = self.global_resolution
            self.stitched_width = self.global_width
            self.stitched_height = self.global_height
            self.stitched_origin_x = self.global_origin_x
            self.stitched_origin_y = self.global_origin_y
            self.get_logger().info("Global costmap initialized.")
        # If origin or size of global costmap changes, resize stitched costmap
        elif (
            self.stitched_width != self.global_width
            or self.stitched_height != self.global_height
            or self.stitched_origin_x != self.global_origin_x
            or self.stitched_origin_y != self.global_origin_y
        ):
            self.resize_stitched_costmap()

    def stitch_local_publish_global(self, msg):
        """
        Callback for subscribing to the local costmap topic.
        Stitches latest local costmap to the global costmap.
        """
        if self.stitched_costmap is None:
            self.get_logger().warn("Stitched costmap not yet initialized.")
            return

        # Extract local costmap data
        local_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        local_resolution = msg.info.resolution
        local_origin_x = msg.info.origin.position.x
        local_origin_y = msg.info.origin.position.y

        # Transform local costmap data to stitched costmap frame
        for y in range(msg.info.height):
            for x in range(msg.info.width):
                # Calculate stitched coordinates
                stitched_x = int(
                    (x * local_resolution + local_origin_x - self.stitched_origin_x) / self.stitched_resolution
                )
                stitched_y = int(
                    (y * local_resolution + local_origin_y - self.stitched_origin_y) / self.stitched_resolution
                )

                # Ensure stitched coordinates are within bounds
                if 0 <= stitched_x < self.stitched_width and 0 <= stitched_y < self.stitched_height:
                    # Update stitched costmap
                    current_value = self.stitched_costmap[stitched_y, stitched_x]
                    new_value = local_data[y, x]

                    if self.use_maximum:
                        self.stitched_costmap[stitched_y, stitched_x] = max(current_value, new_value)
                    else:
                        if new_value > -1:
                            self.stitched_costmap[stitched_y, stitched_x] = new_value

        # Publish the updated global costmap
        self.update_msg = self.global_msg
        self.update_msg.data = self.stitched_costmap.flatten().tolist()
        self.stitched_costmap_publisher.publish(self.update_msg)

    def resize_stitched_costmap(self):
        """Resize the stitched costmap to accommodate new data."""

        # Create a new stitched costmap grid
        new_stitched_costmap = np.full((self.global_height, self.global_width), -1, dtype=np.int8)

        # Calculate the offset for the existing data
        offset_x = int((self.stitched_origin_x - self.global_origin_x) / self.stitched_resolution)
        offset_y = int((self.stitched_origin_y - self.global_origin_y) / self.stitched_resolution)

        # Copy the existing data to the new grid
        for y in range(self.stitched_height):
            for x in range(self.stitched_width):
                new_x = x + offset_x
                new_y = y + offset_y
                if 0 <= new_x < self.global_width and 0 <= new_y < self.global_height:
                    new_stitched_costmap[new_y, new_x] = self.stitched_costmap[y, x]

        # Update the stitched costmap properties
        self.stitched_width = self.global_width
        self.stitched_height = self.global_height
        self.stitched_origin_x = self.global_origin_x
        self.stitched_origin_y = self.global_origin_y

        # Update the stitched costmap
        self.stitched_costmap = new_stitched_costmap
        self.get_logger().info(f"Resized stitched costmap to {self.stitched_width}x{self.stitched_height}")

    def save_costmap_callback(self, request, response):
        """Service callback to save the costmap to a file."""
        if self.update_msg is None:
            response.success = False
            response.message = "No costmap data received yet."
            self.get_logger().warn(response.message)
            return response

        try:
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(f"global_costmap_{current_time}", exist_ok=True)
            pgm_filename = f"global_costmap_{current_time}/costmap.pgm"
            yaml_filename = f"global_costmap_{current_time}/costmap.yaml"

            # Save the costmap to a PGM file
            GlobalCostmapBuilder.save_costmap_to_pgm(self.update_msg, pgm_filename)

            # Save the costmap metadata to a YAML file
            GlobalCostmapBuilder.save_costmap_to_yaml(self.update_msg, yaml_filename)

            response.success = True
            response.message = f"Costmap saved to global_costmap_{current_time}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save costmap: {str(e)}"
            self.get_logger().error(response.message)

        return response

    @staticmethod
    def save_costmap_to_pgm(costmap, filename):
        """Save the costmap data to a PGM file."""
        # Convert the costmap data to a 2D array
        data = np.array(costmap.data, dtype=np.int8).reshape((costmap.info.height, costmap.info.width))

        # Convert cost values to PGM format (0-255)
        data = np.clip(data, 0, 100)  # Clip values to 0-100 (Nav2 costmap range)
        # data = (data * 2.55).astype(np.uint8)  # Scale to 0-255

        # Write the PGM file
        with open(filename, "wb") as pgm_file:
            pgm_file.write(b"P5\n")  # PGM magic number
            pgm_file.write(f"{costmap.info.width} {costmap.info.height}\n".encode())  # Width and height
            pgm_file.write(b"255\n")  # Maximum grayscale value
            pgm_file.write(data.tobytes())  # Binary data

    @staticmethod
    def save_costmap_to_yaml(costmap, filename):
        """Save the costmap metadata to a YAML file."""
        yaml_content = {
            "image": filename.replace(".yaml", ".pgm"),
            "resolution": costmap.info.resolution,
            "origin": [costmap.info.origin.position.x, costmap.info.origin.position.y, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        with open(filename, "w") as yaml_file:
            yaml.dump(yaml_content, yaml_file, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCostmapBuilder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
