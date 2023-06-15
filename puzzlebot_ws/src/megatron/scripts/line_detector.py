#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

frame = None
bridge = CvBridge()
crosswalk = None

#callbacks
def image_callback(img_msg):
    global frame
    frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    #print("imagen recibida")

if __name__ == '__main__':
    rospy.init_node("line_detector")

    pub_x = rospy.Publisher("/error_x", Float32, queue_size = 1)
    pub_w = rospy.Publisher("/error_w", Float32, queue_size = 1)
    pub_c = rospy.Publisher("/crosswalk",Bool, queue_size = 1)
    rospy.Subscriber("/video_source/raw", Image, image_callback)


    rospy.loginfo("Node initialized")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        try:
            frame2 = frame.copy()
            #cv2.imshow('original', frame2)
            #ret, frame = cap.read()
            #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
            #blur = cv2.GaussianBlur(gray,(7, 7),0)
            ret,th1 = cv2.threshold(gray,120,255,cv2.THRESH_BINARY_INV)
            kernel = np.ones((3,3), np.uint8)
            # (height, width)
            th2 = th1[310:, 270:690]
            th2 = cv2.erode(th2, kernel, iterations=5)
            th2 = cv2.dilate(th2, kernel, iterations=9)

            #cv2.imshow('frame2', frame2)

            frame2 = frame2[310:, 270:690]

            contours_blk, hier = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #print(len(contours_blk))
            if(len(contours_blk)>=3):
                crosswalk = True
            else:
                crosswalk = False

            pub_c.publish(crosswalk)

            if len(contours_blk) > 0:	 
                blackbox = cv2.minAreaRect(np.array(contours_blk[0]))
                (x_min, y_min), (w_min, h_min), ang = blackbox
                if ang < -45 :
                    ang = 90 + ang
                if w_min < h_min and ang > 0:	  
                    ang = (90-ang)*-1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang	
                setpoint = 210
                error = int(x_min - setpoint) 
                ang = int(ang)	 
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                cv2.drawContours(frame2,[box],0,(0,0,255),3)	 
                # cv2.putText(frame2,str(ang),(10, 310), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # cv2.putText(frame2,str(error),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                # cv2.line(frame2, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
                # cv2.circle(frame2, (box[0][0], box[0][1]), 1, (255,0,255), 3)
                # cv2.circle(frame2, (box[3][0], box[3][1]), 1, (0,255,255), 3)
                pub_x.publish(error)
                pub_w.publish(ang)
            #print(len(th2))
        
            #cv2.imshow('ddd', th2)
            #cv2.imshow('frame', frame.copy())
            # cv2.imshow('Normal', th2)
            #key = cv2.waitKey(1)
            #if (key & 0xFF == ord('q')):
                #break

        except Exception as error:
            pass

    #video.release()
    #cv2.destroyAllWIndows()

    rate.sleep()