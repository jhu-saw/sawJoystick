# sawJoystick

This SAW component contains code for interfacing with the Linux joystick API.  It compiles on Linux and hasn't been tested on other OSs.

The `ros` folder contains code for a ROS node that interfaces with the sawJoystick component and publishes the analog and digital inputs.  To build the ROS node, make sure you use `catkin build`.

If needed, one can also add OpenIGTLink support using sawOpenIGTLink (contact the sawJoystick developers if you need help with this).

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS (optional)

# Build

You can find some documentation re. compiling cisst and SAW components in the [dVRK wiki](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#catkin-build-and-rosinstall)(best source if you're using Linux with ROS) and the [cisst wiki](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake)(more details and provides instructions for Windows as well).

For Linux with ROS, we provide a rosinstall file to retrieve all the git repositories you need for sawJoystick:
```
wstool merge https://raw.githubusercontent.com/jhu-saw/sawJoystick/devel/ros/joystick.rosinstall
```

# Running the examples

## Linux device

When connecting your joystick to your computer, a pseudo device will be added to the `/dev/input` directory.   Usually something like `/dev/input/js0`, `/dev/input/js1`...  Using the command `dmesg` can help identify which device is used.  Check the file permissions on said device, e.g.,
```sh
ls -al /dev/input/js0
crw-rw-r--+ 1 root input 13, 0 Jan  6 13:39 /dev/input/js0
```
On Ubuntu, the OS usually grants permission to all users to read.   To grant permissions to read and write to the device, use the command `sudo adduser <user_id> input` to add users to the `input` group.   Please note that the user has to logout/login for the new group membership to take effect.

## Main example

The main example provided is `sawJoystickQtExample`.  The command line options are:
```sh
sawJoystickQtExample:
 -j <value>, --json-config <value> : json configuration file (optional)
 -d <value>, --device <value> : device (e.g. /dev/input/js0...) (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
 -m, --component-manager : JSON file to configure component manager (optional)
```

Please note that configuration files are not supported yet.

## ROS

Please read the section above for the configuration file description.  The ROS node is `joystick` and can be found in the package `joystick_ros`:
```sh
roscd saw_joystick_config
rosrun joystick_ros joystick -d /dev/input/js0
```

The ROS node has a few more command line options:
```sh
/home/anton/catkin_ws/devel/lib/joystick_ros/joystick:
 -j <value>, --json-config <value> : json configuration file (optional)
 -d <value>, --device <value> : device (e.g. /dev/input/js0...) (optional)
 -p <value>, --ros-period <value> : period in seconds to read all components and publish (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period (optional)
 -P <value>, --tf-ros-period <value> : period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
 -m, --component-manager : JSON file to configure component manager (optional)
```

Once the node is started AND connected, the following ROS topic should appear:
```sh
/joystick/input_data
```
