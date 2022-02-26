# ign_ros_control

This is a ROS package for integrating the `ros_control` controller architecture with the [Ignition Gazebo](http://ignitionrobotics.org/) simulator.
More information about `ros_control` can be found here: https://control.ros.org/.

This package takes inspiration (and lines of code) from [ign_ros2_control](https://github.com/ignitionrobotics/ign_ros2_control) and [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/noetic-devel/gazebo_ros_control).

#### Demos

You can run some of the example configurations by running the following commands:

```
roslaunch ign_ros_control_demos cartpole.launch controller:=joint_position_controller
roslaunch ign_ros_control_demos cartpole.launch controller:=joint_velocity_controller
roslaunch ign_ros_control_demos cartpole.launch controller:=joint_effort_controller

```
