#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaneSwitchingBot:
    def __init__(self):
        rospy.init_node('lane_switcher', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # State variables
        self.switch_lane_distance = 1.0  # Switch lanes if an obstacle is within 1 meter
        self.lane_width = 0.4  # Lane width in meters (40 cm)
        self.in_lane_one = True  # Track which lane the robot is in
        
        # Updated speeds for stronger motion
        self.forward_velocity = 0.5  # Increased forward velocity
        self.rotation_velocity = 1.0  # Increased angular velocity for turning

    def lidar_callback(self, data):
        """
        Process LiDAR data to detect obstacles and update the robot's state.
        """
        # Split LiDAR ranges into front, left, and right sectors
        front = min(min(data.ranges[0:10] + data.ranges[-10:]), 10)  # Front sector

        # Log the LiDAR readings for debugging
        rospy.loginfo(f"LiDAR Data - Front: {front}")

        # If an obstacle is detected within the switch lane distance
        if front < self.switch_lane_distance:
            rospy.loginfo("Obstacle detected, switching lanes!")
            self.switch_lane()
        else:
            self.move_forward()

    def switch_lane(self):
        """
        Perform a lane switch maneuver using forward and turning motion.
        """
        if self.in_lane_one:
            rospy.loginfo("Switching to lane 2 (left)...")
            # Turn slightly left
            self.twist.angular.z = self.rotation_velocity
            self.twist.linear.x = 0.0
            self.cmd_pub.publish(self.twist)
            rospy.sleep(0.8)  # Rotate for 0.8 seconds (adjusted for faster rotation)

            # Move diagonally forward into the second lane
            self.twist.angular.z = 0.0
            self.twist.linear.x = self.forward_velocity
            self.cmd_pub.publish(self.twist)
            rospy.sleep(1.0)  # Move forward for 1.0 seconds
            self.in_lane_one = False

        else:
            rospy.loginfo("Switching to lane 1 (right)...")
            # Turn slightly right
            self.twist.angular.z = -self.rotation_velocity
            self.twist.linear.x = 0.0
            self.cmd_pub.publish(self.twist)
            rospy.sleep(0.8)  # Rotate for 0.8 seconds (adjusted for faster rotation)

            # Move diagonally forward into the first lane
            self.twist.angular.z = 0.0
            self.twist.linear.x = self.forward_velocity
            self.cmd_pub.publish(self.twist)
            rospy.sleep(1.0)  # Move forward for 1.0 seconds
            self.in_lane_one = True

        # Stop turning and continue forward in the new lane
        self.move_forward()

    def move_forward(self):
        """
        Continue moving forward in the current lane.
        """
        self.twist.linear.x = self.forward_velocity
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def run(self):
        """
        Keep the node running and process callbacks.
        """
        rospy.spin()


if __name__ == '__main__':
    try:
        bot = LaneSwitchingBot()
        bot.run()
    except rospy.ROSInterruptException:
        pass

