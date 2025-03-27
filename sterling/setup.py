from setuptools import find_packages, setup


import glob
import os

package_name = "sterling"
lib_files = [f for f in glob.glob("sterling/lib/sterling/**/*", recursive=True) if os.path.isfile(f)]


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"), [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "params.yaml")]),
        (os.path.join("share", package_name, "config"), [os.path.join("config", "panther_sim.rviz")]),
        (os.path.join("share", package_name, "launch"), [os.path.join("launch", "costmaps.launch.py")]),
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
    maintainer="nchan",
    maintainer_email="nick.chan@utexas.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_costmap_builder = sterling.nodes.global_costmap_builder:main",
            "local_costmap_builder = sterling.nodes.local_costmap_builder:main",
        ],
    },
)
