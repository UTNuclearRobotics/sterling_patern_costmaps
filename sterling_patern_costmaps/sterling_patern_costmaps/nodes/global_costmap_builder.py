import os
from datetime import datetime

import numpy as np
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, TriggerResponse

class GlobalCostmapBuilder(object):
    """
    This class is responsible for managing and building a global costmap by stitching together 
    local costmaps. It listens to topics for local and global costmaps, processes the incoming 
    data, and publishes an updated global costmap. Additionally, it provides a service to save 
    the global costmap to a file.
    """

    def __init__(self):
        # Initialize the ROS1 node
        rospy.init_node("global_costmap_builder", anonymous=True)

        # Declare and get parameters using ROS1 parameter server
        self.sub_topic_local_costmap = rospy.get_param("~sub_topic_local_costmap", "/local_costmap/costmap")
        self.sub_topic_global_costmap = rospy.get_param("~sub_topic_global_costmap", "/global_costmap/costmap")
        self.pub_topic_global_costmap = rospy.get_param("~pub_topic_global_costmap", "global_costmap")
        self.use_maximum = rospy.get_param("~use_maximum", False)

        # Print parameter values
        rospy.logdebug(f"Subscription local costmap topic: {self.sub_topic_local_costmap}")
        rospy.logdebug(f"Subscription global costmap topic: {self.sub_topic_global_costmap}")
        rospy.logdebug(f"Publication global costmap topic: {self.pub_topic_global_costmap}")
        rospy.logdebug(f"Use maximum: {self.use_maximum}")

        # Subscribe to the local costmap
        self.local_sub = rospy.Subscriber(
            self.sub_topic_local_costmap,
            OccupancyGrid,
            self.stitch_local_publish_global,
            queue_size=10
        )

        # Subscribe to the global costmap
        self.global_sub = rospy.Subscriber(
            self.sub_topic_global_costmap,
            OccupancyGrid,
            self.global_costmap_callback,
            queue_size=10
        )

        # Publisher for the global costmap
        self.stitched_costmap_publisher = rospy.Publisher(
            self.pub_topic_global_costmap,
            OccupancyGrid,
            queue_size=10,
            latch=True  # ROS1 equivalent of Transient Local durability
        )

        # Create a service to save the costmap
        self.service = rospy.Service("save_costmap", Trigger, self.save_costmap_callback)

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
            rospy.loginfo("Global costmap initialized.")
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
        Callback function for processing and integrating a local costmap into a global costmap.

        This function subscribes to a local costmap topic, extracts the local costmap data, 
        transforms it into the global costmap frame, and updates the global costmap by stitching 
        the local costmap data into it. The updated global costmap is then published.

        Args:
            msg (OccupancyGrid): ROS message containing the local costmap data. It includes 
                                 metadata such as resolution, origin, width, and height.
        """

        if self.stitched_costmap is None:
            rospy.logwarn("Stitched costmap not yet initialized.")
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
        rospy.loginfo(f"Resized stitched costmap to {self.stitched_width}x{self.stitched_height}")

    def save_costmap_callback(self, request):
        """
        Callback function for a service that saves the current costmap to a file.

        This function is triggered by a service request and attempts to save the 
        current costmap data to a PGM file along with its metadata in a YAML file. 
        The files are stored in a directory named with the current timestamp.
        """
        response = TriggerResponse()

        if self.update_msg is None:
            response.success = False
            response.message = "No costmap data received yet."
            rospy.logwarn(response.message)
            return response

        try:
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(f"global_costmap_{current_time}", exist_ok=True)
            pgm_filename = f"global_costmap_{current_time}/costmap.pgm"
            yaml_filename = f"global_costmap_{current_time}/costmap.yaml"

            # Save the costmap to a PGM file
            self.save_costmap_to_pgm(self.update_msg, pgm_filename)

            # Save the costmap metadata to a YAML file
            self.save_costmap_to_yaml(self.update_msg, yaml_filename)

            response.success = True
            response.message = f"Costmap saved to global_costmap_{current_time}"
            rospy.loginfo(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save costmap: {str(e)}"
            rospy.logerr(response.message)

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
        data = np.where(data == -1, 205, data)  # Unknown cells in ROS1 are 205 in PGM
        data = np.clip(data, 0, 100)  # Clip values to 0-100 (move_base costmap range)
        data = (data * 2.55).astype(np.uint8)  # Scale to 0-255 for PGM

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


def main():
    try:
        GlobalCostmapBuilder()  # Instantiate the node, which sets up subscribers, publishers, etc.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
