import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import tf

from sterling_patern_costmaps.bev import get_BEV_image
from sterling_patern_costmaps.bev_costmap import BEVCostmap
from sterling_patern_costmaps.local_costmap_helper import LocalCostmapHelper

class LocalCostmapBuilder(object):
    """
    This class integrates camera images and local costmap data to generate a terrain-preferred
    local costmap. It subscribes to camera and occupancy grid topics, processes the data using a homography matrix and 
    a terrain representation model, and publishes the updated costmap.
    """    
    
    def __init__(self):
        # Initialize the ROS1 node
        rospy.init_node("local_costmap_builder", anonymous=True)

        # Declare parameters with default values using ROS1 parameter server
        self.sub_topic_camera = rospy.get_param("~sub_topic_camera", "/camera/topic")
        self.sub_topic_local_costmap = rospy.get_param("~sub_topic_local_costmap", "/local_costmap/topic")
        self.pub_topic_local_costmap = rospy.get_param("~pub_topic_local_costmap", "/local_costmap")
        self.pub_topic_local_costmap_hz = rospy.get_param("~pub_topic_local_costmap_hz", 1.0)
        model_path = rospy.get_param("~model_path", "path/to/models")
        self.H = np.array(rospy.get_param("~homography_matrix", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])).reshape(3, 3)
        self.patch_size_px = rospy.get_param("~patch_size_px", 128)
        self.patch_size_m = rospy.get_param("~patch_size_m", 0.23)
        self.base_link_offset_m = rospy.get_param("~base_link_offset_m", 2.8)
        adapted = rospy.get_param("~adapted", False)
        label_obstacles = rospy.get_param("~label_obstacles", False)

        # Print parameter values
        rospy.logdebug(f"Subscription camera topic: {self.sub_topic_camera}")
        rospy.logdebug(f"Subscription local costmap topic: {self.sub_topic_local_costmap}")
        rospy.logdebug(f"Publication local costmap topic: {self.pub_topic_local_costmap}")
        rospy.logdebug(f"Publication local costmap frequency: {self.pub_topic_local_costmap_hz} Hz")
        rospy.logdebug(f"Model path: {model_path}")
        rospy.logdebug(f"Homography matrix: \n{self.H}")
        rospy.logdebug(f"Patch size (px): {self.patch_size_px}")
        rospy.logdebug(f"Patch size (m): {self.patch_size_m}")
        rospy.logdebug(f"Base link offset (m): {self.base_link_offset_m}")
        rospy.logdebug(f"Adapted model: {adapted}")
        rospy.logdebug(f"Label obstacle: {label_obstacles}")

        # Subscribers
        self.camera_subscriber = rospy.Subscriber(
            self.sub_topic_camera,
            Image,
            self.camera_callback,
            queue_size=10
        )
        self.costmap_subscriber = rospy.Subscriber(
            self.sub_topic_local_costmap,
            OccupancyGrid,
            self.costmap_callback,
            queue_size=10
        )

        # Publishers
        self.sterling_patern_costmap_publisher = rospy.Publisher(
            self.pub_topic_local_costmap,
            OccupancyGrid,
            queue_size=10
        )

        # Timer for periodic updates
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_topic_local_costmap_hz), self.update_costmap)

        # Initialize tf listener for ROS1
        self.tf_listener = tf.TransformListener()

        self.get_terrain_preferred_costmap = BEVCostmap(model_path, adapted, label_obstacles).BEV_to_costmap

        self.LocalCostmapHelper = None

        # Buffers to store and fetch latest message
        self.camera_msg = None
        self.yaw_angle = None
        self.occupancy_grid_msg = None

    def camera_callback(self, msg):
        """
        Callback function for processing incoming camera image messages.

        This function is triggered whenever a new image message is received from the camera subscriber.
        It stores the received image message and attempts to compute the yaw angle of the robot by 
        looking up the transform between the "base_link" and "map" frames.

        Args:
            msg (sensor_msgs.msg.Image): The incoming image message from the camera.
        """

        self.camera_msg = msg

        # Lookup transform from base_link to get orientation
        try:
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            self.yaw_angle = euler[2]  # Yaw is the third element (roll, pitch, yaw)
            # rospy.loginfo(f"Yaw angle: {np.degrees(self.yaw_angle)}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return

    def costmap_callback(self, msg):
        """
        Handles incoming OccupancyGrid messages and initializes the LocalCostmapHelper if not already set.

        This method is a callback function for the local costmap subscriber. It processes
        the incoming OccupancyGrid message and updates the local costmap helper object
        with the grid's resolution, width, and height if it has not been initialized yet.
        The received message is stored for further use.

        Args:
            msg (OccupancyGrid): The incoming OccupancyGrid message containing
                                 information about the local costmap.
        """
        
        if self.LocalCostmapHelper is None:
            self.LocalCostmapHelper = LocalCostmapHelper(msg.info.resolution, msg.info.width, msg.info.height)
        self.occupancy_grid_msg = msg

    def update_costmap(self, event=None):
        """
        Updates the local costmap using camera data, yaw angle, and occupancy grid message.
        This function dynamically updates the robot's local costmap with terrain-preferred costs
        based on real-time camera data and orientation.
        """
    
        if not self.camera_msg or not self.yaw_angle or not self.occupancy_grid_msg:
            if self.camera_msg is None:
                rospy.logdebug("Camera message is None")
            if self.yaw_angle is None:
                rospy.logdebug("Yaw angle is None")
            if self.occupancy_grid_msg is None:
                rospy.logdebug("Occupancy grid message is None")
            rospy.loginfo("Waiting for camera and occupancy grid message...")
            return

        # Get BEV image
        image_data = np.frombuffer(self.camera_msg.data, dtype=np.uint8).reshape(
            self.camera_msg.height, self.camera_msg.width, -1
        )
        # Preview the image using OpenCV
        bev_image = get_BEV_image(image_data, self.H, (self.patch_size_px, self.patch_size_px), (7, 12))

        # Get terrain preferred costmap
        terrain_costmap = self.get_terrain_preferred_costmap(bev_image, self.patch_size_px)
        # rospy.loginfo(f"Costmap:\n{terrain_costmap}")

        # TODO: Bug that the costmap is flipped vertically
        #terrain_costmap = np.fliplr(terrain_costmap)

        # Set costs in the region
        data_2d = self.LocalCostmapHelper.set_costs_in_region(
            0, -self.base_link_offset_m, self.patch_size_m, terrain_costmap
        )

        # Rotate the costmap by the yaw angle
        rotated_data = LocalCostmapHelper.rotate_costmap(data_2d, -np.degrees(self.yaw_angle))
        rotated_data = np.array(rotated_data).flatten()

        # Keep the highest cost when stitching the local costmap
        msg = self.occupancy_grid_msg
        msg.data = rotated_data.tolist()
        # msg.data = np.maximum(msg.data, rotated_data).tolist()

        # Publish message
        self.sterling_patern_costmap_publisher.publish(msg)


# class LocalCostmapHelper:
#     def __init__(self, resolution=0.05, width_cells=120, height_cells=120):
#         # Local costmap dimensions and resolution
#         self.resolution = resolution  # 5 cm per cell
#         self.width_cells = width_cells
#         self.height_cells = height_cells
#         self.width_m = width_cells * resolution  # 6 meters
#         self.height_m = height_cells * resolution  # 6 meters

#         # Center of the local costmap in cell coordinates
#         self.center_x = self.width_cells // 2  # 60 cells
#         self.center_y = self.height_cells // 2  # 60 cells

#     def set_costs_in_region(self, x_m, y_m, cell_size_m, terrain_costmap):
#         """
#         Upscale the BEV costs and paste it onto the correct region of the local costmap.
#         Args:
#             x_m: X coordinate in meters
#             y_m: Y coordinate in meters
#             cell_size_m: Cell size in meters
#             terrain_costmap: 2D numpy array
#         Returns:
#             1D numpy array of upscaled costs
#         """
#         upscale_factor = int(cell_size_m / self.resolution)

#         # Convert meters to cells
#         offset_x_cells = int(x_m / self.resolution)
#         offset_y_cells = int(y_m / self.resolution)
#         width_cells = upscale_factor * len(terrain_costmap[0])
#         height_cells = upscale_factor * len(terrain_costmap)

#         # Calculate the bottom-left corner of the region in cell coordinates
#         # x = self.center_x + offset_x_cells
#         x = self.center_x + offset_x_cells - width_cells // 2
#         # y = self.center_y + offset_y_cells
#         y = self.center_y + offset_y_cells - height_cells

#         # Scale the data array to account for the resolution
#         return self.upsample_2d_array(terrain_costmap, upscale_factor, x, y)

#     def upsample_2d_array(self, arr, factor, x_start, y_start):
#         """
#         Upsample a 2D array by a factor.
#         Args:
#             arr (list of list): The original 2D array.
#         Returns:
#             list of list: The upsampled 2D array.
#         """
#         canvas = np.full((self.height_cells, self.width_cells), -1, dtype=int)

#         # Get the dimensions of the original array
#         height = len(arr)
#         width = len(arr[0]) if height > 0 else 0

#         # Fill the upsampled array
#         for i in range(height):
#             for j in range(width):
#                 # Get the value from the original array
#                 value = arr[i][j]

#                 # Fill the corresponding block in the upsampled array
#                 for di in range(factor):
#                     for dj in range(factor):
#                         canvas[y_start + factor * i + di][x_start + factor * j + dj] = value

#         return canvas

#     @staticmethod
#     def quarternion_to_euler(orientation_q):
#         """
#         Convert quaternion to Euler angles
#         Args:
#             orientation_q: Quaternion object
#         Returns:
#             Yaw angle in radians
#         """
#         siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
#         cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
#         yaw_angle = np.arctan2(siny_cosp, cosy_cosp)

#         return yaw_angle

#     @staticmethod
#     def rotate_costmap(data_2d, angle):
#         """
#         Rotate a costmap by a given angle
#         Args:
#             data_2d: 2D numpy array
#             angle: Angle in degrees
#         Returns:
#             rotated_data: 1D list
#         """
#         height, width = data_2d.shape
#         center = (width // 2, height // 2)
#         rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
#         rotated_data = cv2.warpAffine(
#             data_2d,
#             rotation_matrix,
#             (width, height),
#             flags=cv2.INTER_NEAREST,
#             borderMode=cv2.BORDER_CONSTANT,
#             borderValue=-1,
#         )

#         return rotated_data


def main():
    try:
        LocalCostmapBuilder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
