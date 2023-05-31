#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String

bridge = CvBridge()
orig = 0
color_detectado = 0

#range to detect green color
green_lower = np.array([39, 70, 86], np.uint8)
green_upper = np.array([84, 190, 165], np.uint8)

def image_callback(img_msg):
    global orig
    orig = bridge.imgmsg_to_cv2(img_msg, "passthrough")

if __name__=='__main__':
    sub_image=rospy.Subscriber("image", Image, image_callback)
    pub_color=rospy.Publisher("color", String, queue_size=10)
    rospy.loginfo("Node initiated")
    rospy.init_node("image_receiver")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #converting image to hsv
        hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)

        #range 1 to detect red color
        redBajo1 = np.array([0, 100, 20], np.uint8)
        redAlto1 = np.array([15, 255, 255], np.uint8)

        #range 2 to detect red color
        redBajo2 = np.array([170, 100, 20], np.uint8)
        redAlto2 = np.array([180, 255, 255], np.uint8)

        # Red Mask
        maskRed1 = cv2.inRange(hsv, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(hsv, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        maskRedvis = cv2.bitwise_and(orig, orig, mask= maskRed)

        #Green Mask
        maskgreen = cv2.inRange(hsv, green_lower, green_upper)
        maskGreenvis = cv2.bitwise_and(orig, orig, mask= maskgreen)


        # Find contours
        cnts_r = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_r = cnts_r[0] if len(cnts_r) == 2 else cnts_r[1]

        for c in cnts_r:
            perimeter = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)
            if len(approx) > 6:
                cv2.drawContours(orig, [c], -1, (0,0,255), -1)
                #rospy.loginfo("red") 
                pub_color.publish("red")
                color_detectado = 1      


        if(color_detectado == 0):
            cnts_g = cv2.findContours(maskgreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_g = cnts_g[0] if len(cnts_g) == 2 else cnts_g[1]

            
            for c in cnts_g:
                perimeter = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)
                if len(approx) > 6:
                    cv2.drawContours(orig, [c], -1, (36, 255, 12), -1)
                    #rospy.loginfo("green")
                    pub_color.publish("green")
        
        color_detectado = 0
        
        #showing video
        #img_rotate = cv2.rotate(orig, cv2.ROTATE_90_CLOCKWISE)
        #img_rotate2 = cv2.rotate(img_rotate, cv2.ROTATE_90_CLOCKWISE)
        #cv2.imshow('window ', img_rotate2)
        #cv2.imshow('image', orig)

        #cv2.waitKey(1)
        rate.sleep()
