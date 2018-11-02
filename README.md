# Dynamixel Servo Gripper [ROS2]

## Purpose

An end effector is the device at the end of a robotic arm, designed to interact with the environment. To provide for a low cost gripper solution for the underlying project, 2 [dynamixel motors](https://www.trossenrobotics.com/shared/images/PImages/R-903-0188-000-c.jpg) with claws affixed are attached alongside each other. This library (**built & tested with ROS2 bouncy, Ubuntu 16.04**) provides an easy-to-use ROS2 package to control the grip.

## Getting started

* Ensure dynamixel motors have the proper power supply & both servos have a different ID (you can change them by using a [GUI tool](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#gui)
* Install [dynamixel_sdk](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository)
* Download & place these packages in your_ros2_workspace/src
* Run the following commands (root folder of your ROS2 workspace):
```
colcon build --symlink-install
source install/setup.bash
```
* To start using the ROS2 API:
```
dynamixel_gripper_ros2
ros2 topic pub /gripper/command std_msgs/Int32 "{data: 1}" --once
```
This will initialise the gripper node and close the gripper.

## ROS2 API

**Published Topics**
* /gripper/load (msg type: dynamixel_gripper/LoadState)
  - Returns the current load in both servos
* /gripper/state (msg type: dynamixel_gripper/GripState)
  - Returns 0 if gripper is open, and 1 if gripper is closed; contains other motor info as well

**Subscribed Topics**
* /gripper/command (msg type: std_msgs/Int32)
  - 0 - open, 1 - close

## Important points
* Gripper may get hot after prolonged use. Keep track and ensure its **temperature does not exceed 65 degrees celsius**. To track temperature, check published topic **/gripper/state**
* Ensure the dynamixel port in use follows _"/dev/ttyUSB0"_
* Left dynamixel gripper servo should be labeled as ID 1, and right dynamixel gripper servo labeled as ID 2
* ROS1 and ROS2 packages have slightly different msg names (grip_state vs Griptate, load_state vs LoadState)
