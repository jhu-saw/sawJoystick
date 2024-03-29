#
# (C) Copyright 2021-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (joystick VERSION 1.1.0)

# first test for ROS1
find_package (catkin QUIET
  COMPONENTS
  cisst_ros_crtk
  roscpp
  roslib)

# ROS 1
if (catkin_FOUND)
  add_definitions (-DROS1)
  set (ROS_VERSION 1)
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    add_definitions (-DROS2)
    set (ROS_VERSION 2)
    find_package (rclcpp REQUIRED)
    find_package (std_msgs REQUIRED)
    find_package (std_srvs REQUIRED)
    find_package (geometry_msgs REQUIRED)
    find_package (sensor_msgs REQUIRED)
    find_package (diagnostic_msgs REQUIRED)
    find_package (cisst_msgs REQUIRED)
    find_package (tf2_ros REQUIRED)
    find_package (tf2_msgs REQUIRED)
    find_package (crtk_msgs REQUIRED)
    find_package (cisst_ros2_bridge REQUIRED)
    find_package (cisst_ros2_crtk REQUIRED)

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawJoystick has been compiled within cisst, we should find it automatically
  find_package (sawJoystick 1.1.0 REQUIRED)

  if (sawJoystick_FOUND)

    if (ROS_VERSION EQUAL 1)
      file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
      catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
        CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)
      include_directories (${catkin_INCLUDE_DIRS})
    endif ()

    # sawJoystick configuration
    include_directories (${sawJoystick_INCLUDE_DIR} ${joystick_SOURCE_DIR})
    link_directories (${sawJoystick_LIBRARY_DIR})

    add_executable (joystick mainQt.cpp)

    # link against non cisst libraries and saw components
    target_link_libraries (joystick
      ${sawJoystick_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (joystick ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS joystick
      LIBRARY DESTINATION lib/${PROJECT_NAME}
      RUNTIME DESTINATION lib/${PROJECT_NAME}
      LIBRARY DESTINATION lib
      INCLUDES DESTINATION include)

    if (ROS_VERSION EQUAL 2)
      ament_target_dependencies (
        joystick
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        diagnostic_msgs
        cisst_msgs
        tf2_ros
        crtk_msgs
        cisst_ros2_bridge
        cisst_ros2_crtk
        )
      ament_package ()
    endif ()

  endif (sawJoystick_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
