#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

class LaneSwitchingBot:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=10)
        self.state = Int16()

        # State variables
        self.stop_distance = 1.0  # Stop if an obstacle is within 1.5 meters
       # self.safe_distance = 0.5  # Minimum distance to switch lanes
        self.lidar_readings = {}  # Store lidar data for debugging
        self.in_lane_one = True  # Track which lane the robot is in
        rospy.loginfo("Done Init ...")

    def lidar_callback(self, data):
        # Split LiDAR ranges into left, right, and front sectors
        front = min(min(data.ranges[0:10] + data.ranges[-10:]), 10)  # Front sector
        left = min(data.ranges[90:110])  # Left sector
        right = min(data.ranges[270:290])  # Right sector

        # Store readings for debugging
        self.lidar_readings = {"front": front}

        # Check for obstacles in the front sector
        if front < self.stop_distance:
            rospy.loginfo(f"Obstacle detected at {front}m ahead!")
            self.state.data=1
            self.state_pub.publish(self.state)

        else:
            rospy.loginfo(f"Clear path. Front: {front}")
            self.state.data=0
            self.state_pub.publish(self.state)


if __name__ == '__main__':
    rospy.init_node('lane_switcher')
    LaneSwitchingBot()
    rospy.spin() 
