cmake_minimum_required(VERSION 2.8.3)
project(joint_state_publisher_gui)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/joint_state_publisher_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
