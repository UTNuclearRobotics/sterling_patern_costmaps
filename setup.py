from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sterling_patern_costmaps"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test", "test.*"]),
    package_dir={
        package_name: os.path.join("src", package_name),
        "nodes": os.path.join("src", "nodes"),
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py") + glob("launch/*.py")),
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
    maintainer="rlee",
    maintainer_email="ryanlee0518@utexas.edu",
    description="Global and local costmap builder nodes for pattern-based exploration",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_costmap_builder = nodes.global_costmap_builder:main",
            "local_costmap_builder = nodes.local_costmap_builder:main",
        ],
    },
)