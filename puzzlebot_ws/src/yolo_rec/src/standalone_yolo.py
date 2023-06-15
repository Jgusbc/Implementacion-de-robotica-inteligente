#!/usr/bin/env python3
import time
import cv2
import os
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

frame = None

def image_callback(img_msg):
    global frame
    frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")

sys.path.append(sys.path[0]+'/yolov5')
from detection import detector 
from std_msgs.msg import Int32

bridge = CvBridge()

class DetectStop:

    def __init__(self):

        # folder path
        self.weights = "/home/puzzlebot/puzzlebot_ws/src/yolo_rec/src/signals2.pt" # adjust the path to your system! 
        self.yolo = detector(self.weights,0.8)
        # Iterate directory

    def detect(self, source):
        try:
            pred = self.yolo.detect(source)
            for i, det in enumerate(pred):
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    if conf > 0.8:
                        pub.publish(c)
                    else:
                        pub.publish(404)
            # if len(enumerate(pred) == 0):
            #     pub.publish(404)

        except:
            pub.publish(404)

if __name__ == "__main__":
    rospy.init_node("signal")
    rospy.loginfo("Node initialized")
    pub = rospy.Publisher("/signal", Int32, queue_size=1)
    rate = rospy.Rate(100)
    dstop = DetectStop()
    rospy.Subscriber("/video_source/raw", Image, image_callback)

    while not rospy.is_shutdown():
        try:
            dstop.detect(frame)
        except:
            pub.publish(202)

        rate.sleep()

    

