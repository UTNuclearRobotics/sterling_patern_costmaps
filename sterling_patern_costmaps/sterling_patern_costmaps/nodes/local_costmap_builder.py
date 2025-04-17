import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
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
        self.base_link_offset_m = rospy.get_param("~base_link_offset_m", 0.4)
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
            quaternion = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
            self.yaw_angle = LocalCostmapHelper.quarternion_to_euler(quaternion)
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
        # Resize image to 1280x720
        image_data = cv2.resize(
            image_data,
            (1280, 720),  # (width, height)
            interpolation=cv2.INTER_AREA  # Good for downscaling
        )  # Shape: (720, 1280, 3)
        # Preview the image using OpenCV
        bev_image = get_BEV_image(image_data, self.H, (self.patch_size_px, self.patch_size_px), (7, 12))
        # Get terrain preferred costmap
        terrain_costmap = self.get_terrain_preferred_costmap(bev_image, self.patch_size_px)
        # rospy.loginfo(f"Costmap:\n{terrain_costmap}")

        # Set costs in the region
        data_2d = self.LocalCostmapHelper.set_costs_in_region(
            0, -self.base_link_offset_m, self.patch_size_m, terrain_costmap
        )
        # Rotate the costmap by the yaw angle
        rotated_data = LocalCostmapHelper.rotate_costmap(data_2d, -np.degrees(self.yaw_angle) - 90)
        rotated_data = np.array(rotated_data).flatten()
        # Keep the highest cost when stitching the local costmap
        msg = self.occupancy_grid_msg
        msg.data = rotated_data.tolist()
        # msg.data = np.maximum(msg.data, rotated_data).tolist()
        # Publish message
        self.sterling_patern_costmap_publisher.publish(msg)


def main():
    try:
        LocalCostmapBuilder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
