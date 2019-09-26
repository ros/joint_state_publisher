cmake_minimum_required(VERSION 2.8.3)
project(joint_state_publisher)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS joint_state_publisher/joint_state_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/test_mimic_chain.launch)
  add_rostest(test/test_mimic_cycle.launch)
  add_rostest(test/test_zero_joints.launch)
  add_rostest(test/test_multi_joints_urdf.launch)
  add_rostest(test/test_multi_joints_collada.launch)
  add_rostest(test/test_64_joint_robot.launch)

  # Force CMake to find python again in case it was called before to find python 2
  unset(PYTHON_EXECUTABLE CACHE)
  unset(PYTHONINTERP_FOUND CACHE)
  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME test_py3_syntax COMMAND "${PYTHON_EXECUTABLE}" "-m" "flake8" "--select" "E999")
endif()
