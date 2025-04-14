from setuptools import find_packages, setup
import glob
import os

package_name = "sterling_patern_costmaps"
lib_files = [f for f in glob.glob("sterling_patern_costmaps/lib/sterling_patern_costmaps/**/*", recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version="0.1.0",  # Updated to match package.xml
    packages=find_packages(exclude=["test"]),
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "move_base_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "costmap_common_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "global_costmap_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "local_costmap_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "slam_toolbox_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "robot_state_publisher_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "map_server_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "custom_nodes_params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "panther_sim.rviz")]),
        (os.path.join("share", package_name, "launch"), [os.path.join("launch", "costmaps.launch")]),
        (os.path.join("lib", package_name), lib_files),
    ],
    install_requires=[
        "setuptools",
        "opencv-python",
        "numpy",
        "joblib",
        "scikit-learn",
        "torch",
        "pyyaml",
        "tensorflow",
        "torchvision",
    ],
    zip_safe=True,
    maintainer="asharp",
    maintainer_email="asharp@utexas.edu",
    description="Package for converting sterling/patern output into ROS costmaps",  # Updated to match package.xml
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "global_costmap_builder = sterling_patern_costmaps.nodes.global_costmap_builder:main",
            "local_costmap_builder = sterling_patern_costmaps.nodes.local_costmap_builder:main",
        ],
    },
)
