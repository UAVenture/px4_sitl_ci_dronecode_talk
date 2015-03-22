# Testing and analysis #

Copyright (c) 2015 UAVenture AG. All rights reserved.

Repository containing presentation and data for a short talk about SITL testing and analysis with the PX4 autopilot project. Created for the DroneCode Unconference held at the Embedded Linux Conference 2015.

To make the example self contained you'll find some duplicated code from the PX4 repositories.

## Presentation ##

**PDF: **

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

## Links ##


TODO:
- explain repository contents
- add pdf
- add links to relevant projects



