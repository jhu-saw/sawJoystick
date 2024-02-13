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
project (sawJoystick VERSION 1.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawJoystick)
  set (sawJoystick_INCLUDE_DIR
       "${sawJoystick_SOURCE_DIR}/include"
       "${sawJoystick_BINARY_DIR}/include")
  set (sawJoystick_HEADER_DIR "${sawJoystick_SOURCE_DIR}/include/sawJoystick")
  set (sawJoystick_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawJoystick_LIBRARIES sawJoystick)

  # create/configure file for find_package (sawJoystick)
  set (sawJoystick_INCLUDE_DIR
       "${sawJoystick_SOURCE_DIR}/include"
       "${sawJoystick_BINARY_DIR}/include")
  set (sawJoystick_HEADER_DIR "${sawJoystick_SOURCE_DIR}/include/sawJoystick")
  set (sawJoystick_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawJoystick_LIBRARIES sawJoystick)
  include_directories (${sawJoystick_INCLUDE_DIR} ${sawJoystick_BINARY_DIR})

  # add all config files for this component
  cisst_add_config_files (sawJoystick)

  configure_file ("${sawJoystick_SOURCE_DIR}/code/sawJoystickConfig.h.in"
                  "${sawJoystick_BINARY_DIR}/include/sawJoystick/sawJoystickConfig.h")

  add_library (sawJoystick
               ${sawJoystick_HEADER_DIR}/sawJoystickExport.h
               code/mtsJoystick.cpp
               ${sawJoystick_HEADER_DIR}/mtsJoystick.h)

  set_target_properties (sawJoystick PROPERTIES
    VERSION ${sawJoystick_VERSION}
    FOLDER "sawJoystick")

  cisst_target_link_libraries (sawJoystick ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY "${sawJoystick_SOURCE_DIR}/include/sawJoystick"
                     "${sawJoystick_BINARY_DIR}/include/sawJoystick"
           DESTINATION include
           COMPONENT sawJoystick-dev)

  install (TARGETS sawJoystick COMPONENT sawJoystick
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
