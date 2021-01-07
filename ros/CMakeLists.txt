#
# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

project (joystick_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
              roslib
              )

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes
                              cisstParameterTypesQt
			      cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.9 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

  # sawJoystick has been compiled within cisst, we should find it automatically
  find_package (sawJoystick REQUIRED)

  if (sawJoystick_FOUND)

    # sawJoystick configuration
    include_directories (${sawJoystick_INCLUDE_DIR} ${joystick_ros_SOURCE_DIR})
    link_directories (${sawJoystick_LIBRARY_DIR})

    add_executable (joystick mainQt.cpp)

    # link against non cisst libraries and saw components
    target_link_libraries (joystick
      ${sawJoystick_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (joystick ${REQUIRED_CISST_LIBRARIES})

  endif (sawJoystick_FOUND)

endif (cisst_FOUND)
