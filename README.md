# TurtleBot3 Waffle Robot - Lane Keeping and Obstacle Avoidance

This project demonstrates how to achieve straight-line movement and obstacle avoidance using a TurtleBot3 Waffle robot in a Gazebo simulation. The project is divided into two phases: Lane Keeping and Lane Switching.

## Phase 1: Straight Line Movement (Lane Keeping)

In the first phase, the robot uses a camera to calculate its heading and detect any drifting. By analyzing the heading information, the robot identifies deviations and adjusts the steering angle based on contour detection of lines and calculating the centroid of the line.

## Phase 2: Obstacle Avoidance (Lane Switching)

In the second phase, the robot uses a LiDAR sensor to detect obstacles in real-time. When an obstacle is detected at approximately 100 cm, the robot initiates a lane change maneuver to avoid it. The obstacle detection and decision-making algorithms determine the appropriate action based on the sensor data. The robot uses an ultrasonic sensor to identify obstacles and decides to turn left or right to avoid them.

## Prerequisites

1. **Install ROS1**: Follow the instructions to install ROS1 from the official ROS documentation [here](http://wiki.ros.org/noetic/Installation).

2. **Install TurtleBot3**: Follow the instructions to install TurtleBot3 on ROS1 from the official TurtleBot3 documentation [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

## Setup Instructions

### Step 1: Clone the TurtleBot3 Simulation Package

```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
