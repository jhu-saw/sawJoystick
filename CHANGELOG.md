Change log
==========

1.1.0 (2024-01-04)
==================

* API changes:
  * The `component` directory is now moved under `core` to avoid having a `CMakeLists.txt` at the top level (for ROS2/colcon)
* Deprecated features:
  * None
* New features:
  * `ros` subdirectory can now be compiled for ROS1/catkin or ROS2/colcon
* Bug fixes:
  * Fixed CMake to use latest cisst CMake macros, install targets properly defined
