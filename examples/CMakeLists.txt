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

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes
                              cisstParameterTypesQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.9 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawJoystickExamples_IS_CATKIN_BUILT)
  if (sawJoystickExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawJoystick has been compiled within cisst, we should find it automatically
  find_package (sawJoystick REQUIRED)

  if (sawJoystick_FOUND)

    # sawJoystick configuration
    include_directories (${sawJoystick_INCLUDE_DIR})
    link_directories (${sawJoystick_LIBRARY_DIR})

    add_executable (sawJoystickQtExample mainQt.cpp)
    set_property (TARGET sawJoystickQtExample PROPERTY FOLDER "sawJoystick")

    # link against non cisst libraries and saw components
    target_link_libraries (sawJoystickQtExample
                           ${sawJoystick_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawJoystickQtExample ${REQUIRED_CISST_LIBRARIES})

  endif (sawJoystick_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
