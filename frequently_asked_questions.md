# Frequently Asked Questions & Known Issues

This repository contains the configuration and launch files to enable ROS Navigation Stack on AWS DeepRacer, along with the core components to integrate AWS DeepRacer with ROS Navigation stack. As ROS Navigation stack packages are in their development phases, some of the packages for ackermann steering plugins, monocular and stereo visual odometry, laser odometry, etc are yet to be ported over to ROS2 or are not stable enough. 

Here we make notes regarding the areas where further development and support is needed, and the known issues in the simulation and device flow. For detailed information about using ROS Nav2 stack with AWS DeepRacer, see [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

## Sim Flow

As explained in the [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md), we showcase how to use the Nav2 *Navigation2Goal* in the simulation flow to use the ROS Navigation stack packages to autonomously navigate from initial position to goal position. The demo shows one of the ways in which ROS Navigation stack can be used with the AWS DeepRacer simulation artifacts. The document also contains the setup instructions to reproduce the demonstration in AWS Robomaker Integrated Development Environment which has the ROS2 Foxy development setup preinstalled.

**I followed the steps in [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md), but the car in simulation is not moving.**

Verify if there are any errors thrown in the terminal where the launch script is running. Relaunch the nodes by running the commands again.

**I want to modify the plugin configuration, where can I find more details about it.**

More details about ROS Navigation stack plugin configuration is found here: https://navigation.ros.org/configuration/index.html

**I get an TF NAN_INPUT error thrown while running the nav_amcl_demo_sim.launch.py in the simulation.**

```
TF NAN_INPUT: Ignoring transform for child_frame_id "left_front_wheel" from authority "Authority undetectable" because of a nan value in the transform (nan nan nan) (nan nan nan nan)
[bt_navigator-10]          at line 278 in /tmp/binarydeb/ros-foxy-tf2-0.13.10/src/buffer_core.cpp
[bt_navigator-10] Error:   TF DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id "left_front_wheel" from authority "Authority undetectable" because of an invalid quaternion in the transform (nan nan nan nan)
```


This is a known issue and does not impact the movement of the vehicle in simulation. We are investigating the root cause of this error.


## Device Flow

The AWS DeepRacer Evo vehicle is a 1/18th scale Wi-Fi enabled 4-wheel ackermann steering platform that features two RGB cameras and a LiDAR sensor. The LiDAR thats part of the second generation DeepRacer device, is the primary sensor that enables robot localization using global positioning and odometry. More details about the navigation concepts and the state estimation requirements is explained in the [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md). 

The integration with ROS Navigation stack allows the DeepRacer Evo device to be used for mapping using SLAM (Simultaneous Localization and Mapping). The [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md) contains the setup instructions to navigate the physical DeepRacer Evo device by passing in a map. Below are the recommendations for the demo application:

* Make sure to run the Navigation 2 demo on the DeepRacer in the same environment as where the map has been created else the Navigate to Goal may fail due to unsuccessful path planning. Also, finer tuning of the navigation parameters (local / global costmap etc) may be required for successful navigation in the real world.
* The `cmdvel to servo node` will require tuning depending on the map created and `/cmdvel` Twist message output of ROS2 Navigation. Currently we support three action brackets for throttle and steering responses based on the throttle ratios from cmd velocity received. The logic to calculate mapped throttle output and mapped steering output can be modified to satisfy your requirement. (For example: If the map is very small and the velocities generated by the ROS2 Nav Stack as cmdvel output are in a lower scale than the minimum threshold in cmdvel_to_servo_pkg, you can modify the cmdvel_to_servo_pkg to be more responsive to lower velocities and vice versa. More information can be found in the cmdvel_to_servo_pkg and if you want to explore more about how the servo of DeepRacer maps the speed, you can checkout the [Nonlinear speed-mapping equations](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md#nonlinear-speed-mapping-equations). We also recommend [calibrating the DeepRacer](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html) before running ROS2 Nav stack using the DeepRacer application.

**I followed the steps in [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md), but the physical car is not moving.**

Please look at the Device flow section above to get more understanding of how the DeepRacer packages interact with ROS Navigation stack. Possible fixes to get the physical car moving may include recharging the battery, fine tuning action brackets to work with the magnitude of linear velocity passed as part of `/cmdvel` topic and using a better map of the location where the car is run.

**I want to use the DeepRacer to map a new environment. Where can I find the steps to create a map?**
Use the DeepRacer to run SLAM and create a map using navigation packages. Instructions can be found as part of [device flow](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md#part-21--clone-and-build-the-robot-packages-on-the-aws-deepracer-device) section to use SLAM toolbox to create a map.

**Can I use other cameras? Can I use a depth camera for mapping?**
We provide a [mapping](https://github.com/aws-deepracer/aws-deepracer-mapping-sample-project) sample project to showcase the use of Intel RealSense D435 camera. You may have to look into the integration of any/all additional sensors with ROS2 both in simulation as well as on the device.

**Can we use DeepRacer with single front facing camera to navigate autonomously? Is a LiDAR always required?**
The current packages use the LiDAR sensor and the corresponding dependency packages to localize itself in the map provided. The DeepRacer device with the single front facing camera can be used to navigate autonomously by replacing these components to Visual Odometry packages and combine the `/odom` messages from different odometry sources using [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) packages.


Please add any more issues seen with detailed steps to reproduce in the repository issues list.
