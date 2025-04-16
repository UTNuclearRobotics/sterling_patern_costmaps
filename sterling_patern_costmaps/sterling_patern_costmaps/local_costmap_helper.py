import cv2
import numpy as np

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