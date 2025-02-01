#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float64

class camerascans:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.error_pub = rospy.Publisher('/line_error', Float64, queue_size=10)
        self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.read_image)
        self.counter=0

    def read_image(self, message: Image):
        '''
        Reads ROS message and turns it into numpy array.
        It also applies a Gaussian blur and turns the blurred image into HSV color format.
        '''
        self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.image = cv2.resize(self.image, (460, 460), interpolation=cv2.INTER_AREA) 
        #self.blurred_img=cv2.GaussianBlur(self.image,(5,5),1.4)
        blurred_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        self.height, self.width, _ = self.image.shape

        yellow_lower = np.array([20, 40, 90])
        yellow_upper = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(blurred_hsv, yellow_lower, yellow_upper)
        
        contours, _ = cv2.findContours(mask_yellow, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        #find intial Error 
        
        if self.counter < 1 :
            moments = cv2.moments(mask_yellow)
            cx = int(moments['m10']/moments['m00'])
            self.error_init=cx-self.image.shape[1] / 2
            rospy.loginfo("zero error init!")
            self.counter+=1
        
        # Find the largest contour
        if len(contours) > 0:
            moments = cv2.moments(mask_yellow)
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            # Calculate error (deviation from center)
            error=(cx-self.image.shape[1] / 2)-self.error_init
            # Publish error value
            error_msg = Float64()
            error_msg.data = error
            self.error_pub.publish(error_msg)

            rospy.loginfo("Error: %f", error)
            
        elif len(contours) <= 0:
            # Handle case where no line is detected
            rospy.logwarn("Line not detected.")
            error_msg = Float64()
            error_msg.data = 123123.0
            self.error_pub.publish(error_msg)
            
            # Draw results on the image
        cv2.drawContours(self.image, contours, -1, (0, 255, 0), 3)
        cv2.circle(self.image, (int(cx), int(cy)),4, (0, 0, 0), -1)
        cv2.imshow('Camera Output', self.image)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('camera_visualizer')  # Initialize ROS node
    camerascans()
    rospy.spin() 
