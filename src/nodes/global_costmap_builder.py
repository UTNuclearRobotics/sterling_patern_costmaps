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
    This class is responsible for managing and building a global costmap by stitching together 
    local costmaps. It listens to topics for local and global costmaps, processes the incoming 
    data, and publishes an updated global costmap. Additionally, it provides a service to save 
    the global costmap to a file.
    """

    def __init__(self):
        super().__init__("global_costmap_builder")

        # Declare and get parameters
        self.declare_parameter("sub_topic_local_costmap", "/local_costmap/costmap")
        self.declare_parameter("sub_topic_global_costmap", "/global_costmap/costmap")
        self.declare_parameter("pub_topic_global_costmap", "global_costmap")
        self.declare_parameter("use_maximum", False)

        self.sub_topic_local_costmap = self.get_parameter("sub_topic_local_costmap").value
        self.sub_topic_global_costmap = self.get_parameter("sub_topic_global_costmap").value
        self.pub_topic_global_costmap = self.get_parameter("pub_topic_global_costmap").value
        self.use_maximum = self.get_parameter("use_maximum").value

        # Print parameter values
        self.get_logger().debug(f"Subscription local costmap topic: {self.sub_topic_local_costmap}")
        self.get_logger().debug(f"Subscription global costmap topic: {self.sub_topic_global_costmap}")
        self.get_logger().debug(f"Publication global costmap topic: {self.sub_topic_global_costmap}")
        self.get_logger().debug(f"Use maximum: {self.use_maximum}")

        # Subscribe to the local costmap
        self.create_subscription(
            OccupancyGrid,
            self.sub_topic_local_costmap,
            self.stitch_local_publish_global,
            10,
        )

        # Subscribe to the global costmap
        self.create_subscription(
            OccupancyGrid,
            self.sub_topic_global_costmap,
            self.global_costmap_callback,
            10,
        )

        # Publisher for the global costmap
        self.stitched_costmap_publisher = self.create_publisher(
            OccupancyGrid,
            self.pub_topic_global_costmap,
            qos_profile=qos_profile,
        )

        # Create a service to save the costmap
        self.service = self.create_service(Trigger, "save_costmap", self.save_costmap_callback)

        # Initialize stitched costmap
        self.stitched_costmap = None
        self.update_msg = None

    def global_costmap_callback(self, msg):
        """
        Callback function for processing the global costmap message.

        This function is triggered when a new OccupancyGrid message is received. 
        It updates the global costmap properties and initializes or resizes the 
        stitched costmap as needed.

        Args:
            msg (OccupancyGrid): The incoming message containing the global costmap 
                                 data, including resolution, dimensions, and origin.
        """
        
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
        if self.stitched_costmap is None:
            self.get_logger().warn("Stitched costmap not yet initialized.")
            return

        # ─── Extract local data ───────────────────────────────────────
        local_data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        local_res  = msg.info.resolution
        local_ox   = msg.info.origin.position.x
        local_oy   = msg.info.origin.position.y

        stitched_res = self.stitched_resolution
        stitched_ox  = self.stitched_origin_x
        stitched_oy  = self.stitched_origin_y

        h_loc, w_loc = local_data.shape

        # ─── Create local pixel center coordinates (vectorized) ───────
        # Using cell centers gives slightly better behavior than corners in many cases
        # But we still expand to conservative bounds later
        lx = np.arange(w_loc, dtype=np.float32)      # shape (w_loc,)
        ly = np.arange(h_loc, dtype=np.float32)      # shape (h_loc,)

        # World coordinates of **cell centers**
        wx = local_ox + (lx + 0.5) * local_res
        wy = local_oy + (ly + 0.5) * local_res

        # ─── Meshgrid of all cell centers in world frame ─────────────
        WX, WY = np.meshgrid(wx, wy, indexing='xy')   # both (h_loc, w_loc)

        # ─── Transform to stitched grid coordinates ──────────────────
        gx = (WX - stitched_ox) / stitched_res
        gy = (WY - stitched_oy) / stitched_res

        # ─── Conservative bounding box per local cell ────────────────
        # We still expand half cell in each direction → conservative
        half_local_cell = local_res * 0.5 / stitched_res   # in stitched grid units

        gx_min = np.floor(gx - half_local_cell).astype(np.int32)
        gy_min = np.floor(gy - half_local_cell).astype(np.int32)
        gx_max = np.ceil (gx + half_local_cell).astype(np.int32)
        gy_max = np.ceil (gy + half_local_cell).astype(np.int32)

        # ─── Clip indices ─────────────────────────────────────────────
        gx_min = np.clip(gx_min, 0, self.stitched_width)
        gy_min = np.clip(gy_min, 0, self.stitched_height)
        gx_max = np.clip(gx_max, 0, self.stitched_width)
        gy_max = np.clip(gy_max, 0, self.stitched_height)

        # ─── Mask out cells with no overlap ───────────────────────────
        valid = (gx_min < gx_max) & (gy_min < gy_max) & (local_data != -1)
        
        if not np.any(valid):
            return  # nothing to do

        # ─── Only process valid cells ────────────────────────────────
        costs   = local_data[valid]
        gx_min  = gx_min[valid]
        gy_min  = gy_min[valid]
        gx_max  = gx_max[valid]
        gy_max  = gy_max[valid]

        # ─── Now update stitched costmap ─────────────────────────────
        if self.use_maximum:
            # Most common case in costmaps → use np.maximum.at
            for i in range(len(costs)):
                y1, y2 = gy_min[i], gy_max[i]
                x1, x2 = gx_min[i], gx_max[i]
                if y2 <= y1 or x2 <= x1:
                    continue
                np.maximum.at(self.stitched_costmap, 
                            (slice(y1, y2), slice(x1, x2)),
                            costs[i])
        else:
            # Less common branch – still vectorized where possible
            for i in range(len(costs)):
                y1, y2 = gy_min[i], gy_max[i]
                x1, x2 = gx_min[i], gx_max[i]
                if y2 <= y1 or x2 <= x1:
                    continue
                region = self.stitched_costmap[y1:y2, x1:x2]
                mask = region < costs[i]
                region[mask] = costs[i]   # in-place

        # ─── Publish ──────────────────────────────────────────────────
        self.update_msg = self.global_msg
        self.update_msg.data = self.stitched_costmap.flatten().tolist()
        self.stitched_costmap_publisher.publish(self.update_msg)

    def resize_stitched_costmap(self):
        """
        Resizes the stitched global costmap to match the dimensions of the global costmap 
        and accommodates new data while preserving existing data.

        This function creates a new costmap grid with the updated dimensions, calculates 
        the offset for the existing data, and copies the data from the old costmap to the 
        new one. It then updates the stitched costmap properties and logs the resizing 
        operation.
        """

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
        """
        Callback function for a service that saves the current costmap to a file.

        This function is triggered by a service request and attempts to save the 
        current costmap data to a PGM file along with its metadata in a YAML file. 
        The files are stored in a directory named with the current timestamp.
        """
        
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
        """
        Saves a costmap represented as an OccupancyGrid message to a Portable Gray Map (PGM) file.

        This function takes the costmap data, processes it into a 2D array, scales the values 
        to the PGM grayscale range (0-255), and writes it to a PGM file. The PGM format is 
        commonly used for representing grayscale images.

        Args:
            costmap (OccupancyGrid): The costmap data to be saved. It contains metadata such as 
                                     width, height, and the occupancy grid values.
            filename (str): The name of the output PGM file, including the file path.
        """
        
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
        """
        Saves the metadata of a given costmap to a YAML file.

        This function extracts metadata from an OccupancyGrid message and writes it 
        to a YAML file. The metadata includes information such as the resolution, 
        origin, and thresholds for occupied and free spaces.

        Args:
            costmap (OccupancyGrid): The costmap containing metadata to be saved.
            filename (str): The name of the YAML file where the metadata will be saved.
        """

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
