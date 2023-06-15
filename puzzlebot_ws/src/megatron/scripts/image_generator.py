#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=960,
    capture_height=540,
    display_width=960,
    display_height=540,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

if __name__ == '__main__':
    rospy.init_node('image_generator')
    rospy.loginfo('Node initialized')
    pub = rospy.Publisher('/video_source/raw', Image, queue_size=1)

    rate = rospy.Rate(100)
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print('Src opened, %dx%d @ %d fps' % (w, h, fps))

    #Recording video
    #gst_out = "appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! nvv4l2h264enc ! h264parse ! matroskamux ! filesink location=test_semaforo5.mp4 "
    #out = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, float(fps), (int(w), int(h)))

    while not rospy.is_shutdown():
        ret, img = cap.read()
        #cv2.imshow("image", img)
        #cv2.waitKey(2)
        if ret:
            imgMsg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            pub.publish(imgMsg)
            #out.write(img)
            #cv2.imwrite("/home/puzzlebot/puzzlebot_ws/src/megatron/image/img.jpg", img)
        else:
            rospy.loginfo("can't open camera")
        
    #print("Ya termine")
    #out.release()
    cap.release()
    rate.sleep()

