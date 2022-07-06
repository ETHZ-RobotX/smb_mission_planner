cmake_minimum_required(VERSION 3.5)
project(smb_mission_planner)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)
cs_install()
cs_export()

catkin_install_python(PROGRAMS scripts/quaternion_to_euler_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
