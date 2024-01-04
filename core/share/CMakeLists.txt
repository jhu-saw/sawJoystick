cmake_minimum_required (VERSION 3.10)
project(saw_joystick_config VERSION 1.1.0)

find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
endif ()

install (DIRECTORY
  "${dvrk_config_SOURCE_DIR}"
  DESTINATION share/sawJoystick
  COMPONENT sawJoystick-Share
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE)
