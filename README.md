# Testing and analysis #

Copyright (c) 2015 [UAVenture AG](http://www.uaventure.com). All rights reserved.

Repository containing presentation and data for a short talk about SITL testing and analysis with the PX4 autopilot project. Created for the DroneCode Unconference held at the Embedded Linux Conference 2015.

To make the example self contained you'll find some duplicated code from the PX4 repositories.

## Presentation ##

**PDF: [DroneCode Unconference Talk](DroneCode_Unconference_Talk.pdf)**

**Running the vertical takeoff example**

Execution the test with GUI:

```bash
rostest vertical_takeoff sitl.launch gui:=true headless:=false

```

Starting Gazebo and running the test separately:

```bash
roslaunch vertical_takeoff sitl.launch gui:=true headless:=false

# In a second console:
rosrun vertical_takeoff vertical_takeoff_test.py
```

## In this repository ##

- `data`: example test data and charts
- `ros_pkg`: example ROS package with example test code

## Links ##

**PX4**

- [PX4 Autopilot](http://px4.io/)
- [PX4 SITL Testing](https://pixhawk.org/dev/ros/sitl_testing)

**Chart tools**

- [RosbagPandas](https://github.com/aktaylor08/RosbagPandas)
- [Bearcart](https://github.com/wrobstory/bearcart)
