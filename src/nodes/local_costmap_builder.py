import cv2
import numpy as np
import rclpy
import math
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import time

from sterling_patern_costmaps.bev import get_BEV_image, get_BEV_image_gpu
from sterling_patern_costmaps.bev_costmap import BEVCostmap, BEVCostmapGPU


class LocalCostmapBuilder(Node):
    """
    This class integrates camera images and local costmap data to generate a terrain-preferred
    local costmap. It subscribes to camera and occupancy grid topics, processes the data using a homography matrix and 
    a terrain representation model, and publishes the updated costmap.
    """    
    
    def __init__(self):
        super().__init__("local_costmap_builder")

        # Declare parameters with default values
        self.declare_parameter("sub_topic_camera", "/camera/topic")
        self.declare_parameter("sub_topic_local_costmap", "/local_costmap/topic")
        self.declare_parameter("pub_topic_local_costmap", "/local_costmap")
        self.declare_parameter("pub_topic_local_costmap_hz", 10.0)
        self.declare_parameter("pub_topic_bev_img", "bev_img")
        self.declare_parameter("model_path", "path/to/models")
        self.declare_parameter("homography_matrix", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("patch_size_px", 128)
        self.declare_parameter("patch_size_m", 0.25)
        self.declare_parameter("base_link_offset_m", 1.0)
        self.declare_parameter("adapted", False)
        self.declare_parameter("label_obstacles", False)

        # Get parameter values
        self.sub_topic_camera = self.get_parameter("sub_topic_camera").value
        self.sub_topic_local_costmap = self.get_parameter("sub_topic_local_costmap").value
        self.pub_topic_local_costmap_hz = self.get_parameter("pub_topic_local_costmap_hz").value
        self.pub_topic_local_costmap = self.get_parameter("pub_topic_local_costmap").value
        self.pub_topic_bev_img = self.get_parameter("pub_topic_bev_img").value
        model_path = self.get_parameter("model_path").value
        self.H = np.array(self.get_parameter("homography_matrix").value).reshape(3, 3)
        self.patch_size_px = self.get_parameter("patch_size_px").value
        self.patch_size_m = self.get_parameter("patch_size_m").value
        self.base_link_offset_m = self.get_parameter("base_link_offset_m").value
        adapted = self.get_parameter("adapted").value
        label_obstacles = self.get_parameter("label_obstacles").value

        # Print parameter values
        self.get_logger().debug(f"Subscription camera topic: {self.sub_topic_camera}")
        self.get_logger().debug(f"Subscription local costmap topic: {self.sub_topic_local_costmap}")
        self.get_logger().debug(f"Publication local costmap topic: {self.pub_topic_local_costmap}")
        self.get_logger().debug(f"Publication local costmap frequency: {self.pub_topic_local_costmap_hz} Hz")
        self.get_logger().debug(f"Model path: {model_path}")
        self.get_logger().debug(f"Homography matrix: \n{self.H}")
        self.get_logger().debug(f"Patch size (px): {self.patch_size_px}")
        self.get_logger().debug(f"Patch size (m): {self.patch_size_m}")
        self.get_logger().debug(f"Base link offset (m): {self.base_link_offset_m}")
        self.get_logger().debug(f"Adapted model: {adapted}")
        self.get_logger().debug(f"Label obstacle: {label_obstacles}")

        # Subscribers
        self.camera_subscriber = self.create_subscription(Image, self.sub_topic_camera, self.camera_callback, 10)
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid, self.sub_topic_local_costmap, self.costmap_callback, 10
        )

        # Publishers
        self.bridge = CvBridge()
        self.bev_img_publisher = self.create_publisher(Image, self.pub_topic_bev_img, 10)

        self.sterling_patern_costmap_publisher = self.create_publisher(OccupancyGrid, self.pub_topic_local_costmap, 10)

        # Timers
        #self.timer = self.create_timer(self.pub_topic_local_costmap_hz, self.update_costmap)

        # Initialize tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #self.bev_costmap_gpu = BEVCostmapGPU(model_path, adapted, label_obstacles, logger=self.get_logger())
        self.bev_costmap_gpu = BEVCostmapGPU(model_path, adapted, label_obstacles)
        self.get_terrain_preferred_costmap_gpu = self.bev_costmap_gpu.BEV_to_costmap_gpu

        self.LocalCostmapHelper = None

        # Buffers to store and fetch latest message
        self.camera_msg = None
        self.yaw_angle = None
        self.occupany_grid_msg = None

    def camera_callback(self, msg):
        """
        Callback function for processing incoming camera image messages.
        Now processes the image immediately instead of buffering.
        """
        # Lookup transform from base_link to get orientation
        try:
            transform = self.tf_buffer.lookup_transform("panther/base_link", "map", rclpy.time.Time())
            yaw_angle = LocalCostmapHelper.quarternion_to_euler(transform.transform.rotation)
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # Check if we have occupancy grid
        if self.occupany_grid_msg is None:
            self.get_logger().debug("Waiting for occupancy grid message...")
            return

        # Process the image immediately
        self.process_camera_image(msg, yaw_angle)

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
        self.occupany_grid_msg = msg

    def process_camera_image(self, camera_msg, yaw_angle):
        """
        Process a camera image and update the costmap.
        Called directly from camera_callback for each new image.
        """
        start = time.time()
        
        # Get image data
        image_data = np.frombuffer(camera_msg.data, dtype=np.uint8).reshape(
            camera_msg.height, camera_msg.width, -1
        )
        
        # Generate BEV on GPU and keep it there
        gpu_bev_image = get_BEV_image_gpu(
            image_data, 
            self.H, 
            (self.patch_size_px, self.patch_size_px), 
            (7, 12),
            logger=None,
            return_gpu=True
        )
        
        t1 = time.time()
        
        # Publish BEV image for visualization (needs CPU version)
        bev_cpu = gpu_bev_image.download()
        ros_image = self.bridge.cv2_to_imgmsg(bev_cpu, encoding="bgr8")
        self.bev_img_publisher.publish(ros_image)
        
        # Get terrain preferred costmap (GPU pipeline)
        terrain_costmap = self.get_terrain_preferred_costmap_gpu(gpu_bev_image, self.patch_size_px)
        
        t2 = time.time()
        
        self.get_logger().info(f"BEV generation (GPU): {(t1-start)*1000:.2f}ms")
        self.get_logger().info(f"Costmap inference (GPU): {(t2-t1)*1000:.2f}ms")
        
        # TODO: Bug that the costmap is flipped horizontally
        terrain_costmap = np.fliplr(terrain_costmap)

        # Set costs in the region
        data_2d = self.LocalCostmapHelper.set_costs_in_region(
            0, -self.base_link_offset_m, self.patch_size_m, terrain_costmap
        )

        # Rotate the costmap by the yaw angle
        rotated_data = LocalCostmapHelper.rotate_costmap(data_2d, np.degrees(yaw_angle) - 90)
        rotated_data = np.array(rotated_data).flatten()

        # Publish message
        msg = self.occupany_grid_msg
        msg.data = rotated_data.tolist()
        self.sterling_patern_costmap_publisher.publish(msg)

    def update_costmap(self):
        """
        Updates the local costmap using camera data, yaw angle, and occupancy grid message.
        This function is dynamically updates the robot's local costmap with terrain-preferred costs
        based on real-time camera data and orientation.
        """
    
        if not self.camera_msg or not self.yaw_angle or not self.occupany_grid_msg:
            if self.camera_msg is None:
                self.get_logger().debug("Camera message is None")
            if self.yaw_angle is None:
                self.get_logger().debug("Yaw angle is None")
            if self.occupany_grid_msg is None:
                self.get_logger().debug("Occupancy grid message is None")
            self.get_logger().info("Waiting for camera and occupancy grid message...")
            return

        # Get BEV image
        image_data = np.frombuffer(self.camera_msg.data, dtype=np.uint8).reshape(
            self.camera_msg.height, self.camera_msg.width, -1
        )
        start = time.time()
    
        # Get image data
        image_data = np.frombuffer(self.camera_msg.data, dtype=np.uint8).reshape(
            self.camera_msg.height, self.camera_msg.width, -1
        )
        
        # Generate BEV on GPU and keep it there
        gpu_bev_image = get_BEV_image_gpu(
            image_data, 
            self.H, 
            (self.patch_size_px, self.patch_size_px), 
            (7, 12),
            logger=self.get_logger(),
            return_gpu=True  # Keep on GPU!
        )
        
        t1 = time.time()
        
        # Publish BEV image for visualization (needs CPU version)
        bev_cpu = gpu_bev_image.download()
        ros_image = self.bridge.cv2_to_imgmsg(bev_cpu, encoding="bgr8")
        self.bev_img_publisher.publish(ros_image)
        
        # Get terrain preferred costmap (GPU pipeline)
        terrain_costmap = self.get_terrain_preferred_costmap_gpu(gpu_bev_image, self.patch_size_px)
        
        t2 = time.time()
        
        self.get_logger().info(f"BEV generation (GPU): {(t1-start)*1000:.2f}ms")
        self.get_logger().info(f"Costmap inference (GPU): {(t2-t1)*1000:.2f}ms")
        
        # TODO: Bug that the costmap is flipped horizontally
        terrain_costmap = np.fliplr(terrain_costmap)

        # Set costs in the region
        data_2d = self.LocalCostmapHelper.set_costs_in_region(
            0, -self.base_link_offset_m, self.patch_size_m, terrain_costmap
        )

        # Rotate the costmap
        rotated_data = LocalCostmapHelper.rotate_costmap(data_2d, np.degrees(self.yaw_angle) - 90)
        rotated_data = np.array(rotated_data).flatten()

        # Publish
        msg = self.occupany_grid_msg
        msg.data = rotated_data.tolist()
        self.sterling_patern_costmap_publisher.publish(msg)


class LocalCostmapHelper:
    def __init__(self, resolution=0.05, width_cells=120, height_cells=120):
        # Local costmap dimensions and resolution
        self.resolution = resolution  # 5 cm per cell
        self.width_cells = width_cells
        self.height_cells = height_cells
        self.width_m = width_cells * resolution  # 6 meters
        self.height_m = height_cells * resolution  # 6 meters

        # Center of the local costmap in cell coordinates
        self.center_x = self.width_cells // 2  # 60 cells
        self.center_y = self.height_cells // 2  # 60 cells

    def set_costs_in_region(self, x_m, y_m, cell_size_m, terrain_costmap):
        """
        Upscale the BEV costs and paste it onto the correct region of the local costmap.
        Args:
            x_m: X coordinate in meters
            y_m: Y coordinate in meters
            cell_size_m: Cell size in meters
            terrain_costmap: 2D numpy array
        Returns:
            1D numpy array of upscaled costs
        """
        upscale_factor = int(cell_size_m / self.resolution)

        # Convert meters to cells
        offset_x_cells = int(x_m / self.resolution)
        offset_y_cells = int(y_m / self.resolution)
        width_cells = upscale_factor * len(terrain_costmap[0])
        height_cells = upscale_factor * len(terrain_costmap)

        # Calculate the bottom-left corner of the region in cell coordinates
        # x = self.center_x + offset_x_cells
        x = self.center_x + offset_x_cells - width_cells // 2
        # y = self.center_y + offset_y_cells
        y = self.center_y + offset_y_cells - height_cells

        # Scale the data array to account for the resolution
        return self.upsample_2d_array(terrain_costmap, upscale_factor, x, y)

    def upsample_2d_array(self, arr, factor, x_start, y_start):
        """
        Upsample a 2D array by a factor.
        Args:
            arr (list of list): The original 2D array.
        Returns:
            list of list: The upsampled 2D array.
        """
        canvas = np.full((self.height_cells, self.width_cells), -1, dtype=int)

        # Get the dimensions of the original array
        height = len(arr)
        width = len(arr[0]) if height > 0 else 0

        # Fill the upsampled array
        for i in range(height):
            for j in range(width):
                # Get the value from the original array
                value = arr[i][j]

                # Fill the corresponding block in the upsampled array
                for di in range(factor):
                    for dj in range(factor):
                        canvas[y_start + factor * i + di][x_start + factor * j + dj] = value

        return canvas

    @staticmethod
    def quarternion_to_euler(orientation_q):
        """
        Convert quaternion to Euler angles
        Args:
            orientation_q: Quaternion object
        Returns:
            Yaw angle in radians
        """
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw_angle = np.arctan2(siny_cosp, cosy_cosp)

        return yaw_angle

    @staticmethod
    def rotate_costmap(data_2d, angle):
        """
        Rotate a costmap by a given angle
        Args:
            data_2d: 2D numpy array
            angle: Angle in degrees
        Returns:
            rotated_data: 1D list
        """
        height, width = data_2d.shape
        center = (width // 2, height // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_data = cv2.warpAffine(
            data_2d,
            rotation_matrix,
            (width, height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=-1,
        )

        return rotated_data


def main(args=None):
    rclpy.init(args=args)
    costmap_updater = LocalCostmapBuilder()
    rclpy.spin(costmap_updater)
    costmap_updater.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
