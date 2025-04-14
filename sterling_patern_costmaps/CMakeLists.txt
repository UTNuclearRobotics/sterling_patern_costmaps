cmake_minimum_required(VERSION 3.0.2)
project(sterling_patern_costmaps)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS rclpy
)
catkin_python_setup()

## Specify additional locations of header files (not needed for Python-only packages)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

## Install Python scripts (nodes)
catkin_install_python(PROGRAMS
  sterling_patern_costmaps/nodes/global_costmap_builder.py
  sterling_patern_costmaps/nodes/local_costmap_builder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and config files
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Add tests (optional, can be expanded later)
# if(CATKIN_ENABLE_TESTING)
#   find_package(roslint REQUIRED)
#   roslint_python(nodes/global_costmap_builder.py nodes/local_costmap_builder.py)
# endif()