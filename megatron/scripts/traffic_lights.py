#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

orig = 0
bridge = CvBridge()

#range 1 to detect red color
red_lower = np.array([92, 114, 170], np.uint8)
red_upper = np.array([255, 255, 255], np.uint8)

#range to detect green color
green_lower = np.array([42, 195, 0], np.uint8)
green_upper = np.array([146, 255, 255], np.uint8)

#range to detect yellow color
yellow_lower = np.array([10, 10, 175], np.uint8)
yellow_upper = np.array([55, 255, 255], np.uint8)

# Configurar parametros de colores
# cv2.namedWindow('window')

# def nothing(x):
# 	pass

# cv2.createTrackbar('h', 'window', 0, 255, nothing)
# cv2.createTrackbar('s', 'window', 0, 255, nothing)
# cv2.createTrackbar('v', 'window', 0, 255, nothing)
# cv2.createTrackbar('h_2', 'window', 0, 255, nothing)
# cv2.createTrackbar('s_2', 'window', 0, 255, nothing)
# cv2.createTrackbar('v_2', 'window', 0, 255, nothing)

#callbacks
def image_callback(img_msg):
    global orig
    orig = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    #print("imagen recibida")

if __name__=='__main__':
    rospy.init_node("traffic_lights")
    pub_color = rospy.Publisher("color", String, queue_size=10)
    rospy.Subscriber("image", Image, image_callback)

    rospy.loginfo("Node initialized")
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        area_threshold_yellow = 125
        area_threshold_green = 450
        area_threshold_red = 350
        

        try: 
            #ret, orig = cap.read()
            #converting image to hsv
            hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)

            #configurar parametros colores
            # h = cv2.getTrackbarPos('h', 'window')
            # s = cv2.getTrackbarPos('s', 'window')
            # v = cv2.getTrackbarPos('v', 'window')
            # h_2 = cv2.getTrackbarPos('h_2', 'window')
            # s_2 = cv2.getTrackbarPos('s_2', 'window')
            # v_2 = cv2.getTrackbarPos('v_2', 'window')

            #range to detect yellow color
            # yellow_lower = np.array([h, s, v], np.uint8)
            # yellow_upper = np.array([h_2, s_2, v_2], np.uint8)

            # Red Mask
            maskRed = cv2.inRange(hsv, red_lower, red_upper)
            maskRedvis = cv2.bitwise_and(orig, orig, mask= maskRed)

            #Green Mask
            maskgreen = cv2.inRange(hsv, green_lower, green_upper)
            maskGreenvis = cv2.bitwise_and(orig, orig, mask= maskgreen)

            #Yellow Mask
            maskyellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
            maskYellowvis = cv2.bitwise_and(orig, orig, mask= maskyellow)

            # Find red contours
            cnts_r = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_r = cnts_r[0] if len(cnts_r) == 2 else cnts_r[1]

            for c in cnts_r:
                area = cv2.contourArea(c)
                if area > area_threshold_red: 
                    pub_color.publish("red")
                    print("red")  

            # Find yellow contours
            cnts_y = cv2.findContours(maskyellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_y = cnts_y[0] if len(cnts_y) == 2 else cnts_y[1]

            for c in cnts_y:
                area = cv2.contourArea(c)
                if area > area_threshold_yellow:
                    pub_color.publish("yellow") 
                    print("yellow")    

            # Find green contours
            cnts_g = cv2.findContours(maskgreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_g = cnts_g[0] if len(cnts_g) == 2 else cnts_g[1]

            for c in cnts_g:
                area = cv2.contourArea(c)
                if area > area_threshold_green: 
                    pub_color.publish("green")
                    print("green")   
            
            print("__")
            #cv2.imshow('green ', maskRedvis)
            #cv2.imshow('green ', maskGreenvis)
            cv2.imshow('window', maskYellowvis)
            #cv2.imshow('image', orig)
            cv2.waitKey(1)
        except:
            print("error")   

            #showing video
        
        
        rate.sleep()
