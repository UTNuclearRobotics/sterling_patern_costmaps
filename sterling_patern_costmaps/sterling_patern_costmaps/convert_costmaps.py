"""
This file converts diffent kind of costmaps.

The main functionalities include:
1. Convert ROS1 costmap in the form of an OccupancyGrid into another msg type.
"""

import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class ConvertCostmaps:
    """
    Class to process a ROS1 costmaps in the form of an OccupancyGrid into another msg type.
    """

    def __init__(self):
        self.ros1_costmap = OccupancyGrid
        self.ros2_costmap = OccupancyGrid
        self.other_costmap = None

    @staticmethod
    def ros1_to_other(self, ros1_costmap):
        """Convert from a ROS1 costmap into another msg type."""
        other_costmap = None
        other_costmap.header = ros1_costmap.header
        other_costmap.type = "other"
        other_costmap.rect = np.array([])
        other_costmap.resolution = float(ros1_costmap.info.resolution)
        other_costmap.rows = int(ros1_costmap.info.height)
        other_costmap.columns = int(ros1_costmap.info.width)
        for cell in ros1_costmap.data:
            other_costmap.data.append(float(cell))

        return other_costmap

    @staticmethod
    def other_to_ros1(self, other_costmap):
        """Convert from a ROS1 costmap into another msg type."""
        ros1_costmap = OccupancyGrid
        ros1_costmap.header = other_costmap.header
        ros1_costmap.info.map_load_time = other_costmap.header.stamp
        ros1_costmap.info.resolution = other_costmap.resolution
        ros1_costmap.info.width = other_costmap.columns
        ros1_costmap.info.height = other_costmap.rows
        ros1_costmap.info.origin = Pose
        for cell in other_costmap.data:
            ros1_costmap.data.append(float(cell))

        return ros1_costmap

    @staticmethod
    def ros1_to_ros2(self, ros1_costmap):
        """Convert from a ROS1 costmap into a ROS2 msg type."""
        ros2_costmap = ros1_costmap

        return ros2_costmap
