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

project (sawJoystick)

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawJoystick_IS_CATKIN_BUILT)
  if (sawJoystick_IS_CATKIN_BUILT)
    set (sawJoystick_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawJoystick/cmake")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    set (sawJoystick_CONFIG_FILE_DIR "${sawJoystick_BINARY_DIR}")
  endif ()

  # create/configure file for find_package (sawJoystick)
  set (sawJoystick_INCLUDE_DIR
       "${sawJoystick_SOURCE_DIR}/include"
       "${sawJoystick_BINARY_DIR}/include")
  set (sawJoystick_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawJoystick_LIBRARIES sawJoystick)

  # Set the version number
  set (sawJoystick_VERSION_MAJOR "1")
  set (sawJoystick_VERSION_MINOR "0")
  set (sawJoystick_VERSION_PATCH "0")
  set (sawJoystick_VERSION "${sawJoystick_VERSION_MAJOR}.${sawJoystick_VERSION_MINOR}.${sawJoystick_VERSION_PATCH}")

  # Generate sawJoystickRevision.h
  configure_file ("${sawJoystick_SOURCE_DIR}/code/sawJoystickRevision.h.in"
                  "${sawJoystick_BINARY_DIR}/include/sawJoystick/sawJoystickRevision.h")

  include_directories (${sawJoystick_INCLUDE_DIR} ${sawJoystick_BINARY_DIR})
  set (sawJoystick_HEADER_DIR "${sawJoystick_SOURCE_DIR}/include/sawJoystick")

  add_library (sawJoystick
               ${sawJoystick_HEADER_DIR}/sawJoystickExport.h
               code/mtsJoystick.cpp
               ${sawJoystick_HEADER_DIR}/mtsJoystick.h)
  set_property (TARGET sawJoystick PROPERTY FOLDER "sawJoystick")
  cisst_target_link_libraries (sawJoystick ${REQUIRED_CISST_LIBRARIES})

  set (sawJoystick_CMAKE_CONFIG_FILE
       "${sawJoystick_CONFIG_FILE_DIR}/sawJoystickConfig.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawJoystickConfig.cmake.in"
                  "${sawJoystick_CMAKE_CONFIG_FILE}"
                  @ONLY)

  configure_file ("${sawJoystick_SOURCE_DIR}/code/sawJoystickConfig.h.in"
                  "${sawJoystick_BINARY_DIR}/include/sawJoystick/sawJoystickConfig.h")

  # Install target for headers and library
  install (DIRECTORY
           "${sawJoystick_SOURCE_DIR}/include/sawJoystick"
           "${sawJoystick_BINARY_DIR}/include/sawJoystick"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawJoystick
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawJoystick_BINARY_DIR}/sawJoystickConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
