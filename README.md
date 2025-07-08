# TurtleBot3 Waffle Robot - autonomous Lane Tracking & obstacle Avoidance Robot

This project demonstrates how to achieve straight-line movement and obstacle avoidance using a TurtleBot3 Waffle robot in a Gazebo simulation. The project is divided into two phases: Lane Keeping and Lane Switching.

## Phase 1: Straight Line Movement (Lane Keeping)

In the first phase, the robot uses a camera to calculate its heading and detect any drifting. By analyzing the heading information, the robot identifies deviations and adjusts the steering angle based on contour detection of lines and calculating the centroid of the line.
![WhatsApp Image 2024-11-27 at 10 30 24_847cc365](https://github.com/user-attachments/assets/da497cc5-13cb-49db-864e-9795049ce9a9)

## Phase 2: Obstacle Avoidance (Lane Switching)

In the second phase, the robot uses a LiDAR sensor to detect obstacles in real-time. When an obstacle is detected at approximately 100 cm, the robot initiates a lane change maneuver to avoid it. The obstacle detection and decision-making algorithms determine the appropriate action based on the sensor data. The robot uses an ultrasonic sensor to identify obstacles and decides to turn left or right to avoid them.
![WhatsApp Image 2024-11-27 at 10 30 23_9b352f1c](https://github.com/user-attachments/assets/4baa2082-5237-48e4-b19e-42c757e2f47b)

## Prerequisites

1. **Install ROS1**: Follow the instructions to install ROS1 from the official ROS documentation [here](http://wiki.ros.org/noetic/Installation).

2. **Install TurtleBot3**: Follow the instructions to install TurtleBot3 on ROS1 from the official TurtleBot3 documentation [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

## Setup Instructions

### Step 1: Clone the TurtleBot3 Simulation Package

```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
### Step 2:Add the World Files
Add the two world files (lane_keeping.world and lane_switching.world) to the TurtleBot3 simulation package.
```bash
cp /path/to/lane_keeping.world ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/
cp /path/to/lane_switching.world ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/
```

### Step 3: Add the Launch Files
Add the two launch files (lane_keeping.launch and lane_switching.launch) to the TurtleBot3 simulation package.
```bash
cp /path/to/lane_keeping.launch ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/
cp /path/to/lane_switching.launch ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/
```
### Step 4: Add the Python Scripts
Add the source Python scripts (lane_keeping.py and lane_switching.py) to the TurtleBot3 package.
```bash
cp /path/to/lane_keeping.py ~/catkin_ws/src/turtlebot3/turtlebot3_example/scripts/
cp /path/to/lane_switching.py ~/catkin_ws/src/turtlebot3/turtlebot3_example/scripts/
```
### Step 5: Build the Workspace
Navigate to your catkin workspace and build the packages.
```bash
cd ~/catkin_ws
catkin_make
```
### Step 6: Launch the Simulation
To launch the lane-keeping simulation:
```bash
roslaunch turtlebot3_gazebo lane_keeping.launch
```
To launch the lane-switching simulation:
```bash
roslaunch turtlebot3_gazebo lane_switching.launch
```
### Step 7: Run the Scripts
To Run the lane-keeping Control:
```bash
rosrun turtlebot3_gazebo camera_scans.py
```
```bash
rosrun turtlebot3_gazebo control.py
```
To Run the lane-switching Control:
```bash
rosrun turtlebot3_gazebo camera_scans.py
```
```bash
rosrun turtlebot3_gazebo control2.py
```
```bash
rosrun turtlebot3_gazebo object_detection.py
```
```bash
rosrun turtlebot3_gazebo lane_switching_bot.py
```

Link to video of the project : [here] (https://www.linkedin.com/posts/shehab-abdo-a94946198_robotics-ros-gazebo-activity-7291479772294004737-o52V?utm_source=share&utm_medium=member_desktop&rcm=ACoAAC5p3pABUmb1OvPQW8oWWD1ArLuAKxEYADY)
