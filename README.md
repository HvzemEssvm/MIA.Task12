# MIA Task 12 (This README will be changed)
---
## This repository includes the following⬇:
1. PID Directory Named `turtlebot_pid`
2. Kinematics Directory Named `kinematics`
3. PID-turtlebot branch includes the package of PID controlling but with dynamic changing for K values (It isn't working)
4. Branch Named ``Select_Motors`` Regarding Task 12.2

### PID Controlling
The package includes a Python script which control the movement of the robot to go to a specific target position and decrease its velocity near the target position until it stops.

**To Run This file Follow These Steps:**
- Open 3 Terminals.
- **First Terminal:** 
```
$ Write roscore
```
- **Second Terminal:** 
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
- **Third Terminal:** 
```
$ cd <your workspace>
$ catkine_make
$ rosrun turtlebot_pid pid.py
```
Now you can see the robot move in gazebo towards its target position
