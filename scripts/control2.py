#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from camera_scans import camerascans
from std_msgs.msg import Int16
class LineFollower:
    def __init__(self):
        self.suberror=rospy.Subscriber("/line_error", Float64, self.control_fn)
        self.velpub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_selection)
        self.velocity = Twist()
        self.condition=0
        self.object=1
         # PID gains
        self.Kp = 0.003
        self.rate = rospy.Rate(1)

    def state_selection(self,state):
        self.condition=state.data
        rospy.loginfo("The current state is %d",self.condition)


    def control_fn(self,error_msg):
        if self.condition == 0:
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
                self.velocity.linear.x=0.3
                self.velocity.angular.z = 0.0
                self.velpub.publish(self.velocity)

            while error ==123123.0 :
                rospy.loginfo("We Have Arrived!")
                self.velocity.linear.x=0.0
                self.velocity.angular.z = 0.0
                self.velpub.publish(self.velocity)
            
        else :
            rospy.loginfo("Object Detected!")
            self.velocity.linear.x=0.0
            self.velocity.angular.z = 0.8
            self.velpub.publish(self.velocity)
           # self.rate.sleep()

            while (self.condition==1 and self.object==1):     #condition for passing first object
                self.velocity.linear.x=0.3
                self.velocity.angular.z = 1.1
                self.velpub.publish(self.velocity)
                self.rate.sleep()

            self.object=2
            while (self.condition==1 and self.object==2):     #condition for passing 2nd object
                self.velocity.linear.x=0.5
                self.velocity.angular.z = -1.1
                self.velpub.publish(self.velocity)
                self.rate.sleep()
            self.object=3
if __name__ == '__main__':
    rospy.init_node('line_follower')
    LineFollower()
    rospy.spin()
