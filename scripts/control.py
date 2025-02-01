#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        self.suberror=rospy.Subscriber("/line_error", Float64, self.control_fn)
        self.velpub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity = Twist()
         # PID gains
        self.Kp = 0.003

    def control_fn(self,error_msg):
        error = error_msg.data
        if error !=0 :

            proportional = self.Kp * error
            control_output = proportional

            self.velocity.linear.x=0.2
            self.velocity.angular.z = -control_output
            
            self.velpub.publish(self.velocity)
            rospy.loginfo("Error: %f, Angular Velocity: %f,Linear Velocity: %f", error_msg.data, self.velocity.angular.z,self.velocity.linear.x)
        elif error ==0 :
            rospy.loginfo("Zero Error,we are going faster !")
            self.velocity.linear.x=0.5
            self.velocity.angular.z = 0.0
            self.velpub.publish(self.velocity)

        while error ==123123.0 :
            rospy.loginfo("We Have Arrived!")
            self.velocity.linear.x=0.0
            self.velocity.angular.z = 0.0
            self.velpub.publish(self.velocity)

if __name__ == '__main__':
    rospy.init_node('line_follower')
    LineFollower()
    rospy.spin()
