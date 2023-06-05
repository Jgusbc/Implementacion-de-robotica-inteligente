#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32

msg = String()

blanco_izq = 0.0
blanco_der = 0.0

rospy.init_node("detector")
rospy.loginfo("Node initiated")
pub = rospy.Publisher("/direction", String, queue_size = 10)
pub_izq = rospy.Publisher("/blanco_der", Float32, queue_size = 10)
pub_der = rospy.Publisher("/blanco_izq", Float32, queue_size = 10)

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
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

#cap = cv2.VideoCapture('res/video_pista.mp4')
dispW = 320
dispH = 240
flip = 2
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

if __name__ == '__main__':
    while not rospy.is_shutdown():

        ret, frame = cap.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #blur = cv2.GaussianBlur(gray,(7, 7),0)
        ret,th1 = cv2.threshold(gray,120,255,cv2.THRESH_BINARY_INV)
        kernel = np.ones((3,3), np.uint8)
        # (height, width)
        th2 = th1[340:, 230:730]
        th2 = cv2.erode(th2, kernel, iterations=5)
        th2 = cv2.dilate(th2, kernel, iterations=9)

        #cv2.imshow('frame1', frame)

        frame = frame[340:, 230:730]

        contours_blk, hier = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_blk) > 0:	 
            blackbox = cv2.minAreaRect(np.array(contours_blk[0]))
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45 :
                ang = 90 + ang
            if w_min < h_min and ang > 0:	  
                ang = (90-ang)*-1
            if w_min > h_min and ang < 0:
                ang = 90 + ang	  
            setpoint = 320
            error = int(x_min - setpoint) 
            ang = int(ang)	 
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(frame,[box],0,(0,0,255),3)	 
            #cv2.putText(frame,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame,str(error),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.line(frame, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
        #print(len(th2))
    
        blanco_izq = 0.0
        blanco_der = 0.0

        range_left_min = 0
        range_left_max = 75
        range_right_min = 425
        range_right_max = 500

        #Numero de pixeles blancos del lado izquierdo
        for i in range (0,50):
            for j in range (range_left_min,range_left_max):
                if (th2[i][j] == 255):
                    blanco_izq += 1
        for i in range (150,200):
            for j in range (range_left_min,range_left_max):
                if (th2[i][j] == 255):
                    blanco_izq += 1

        #Numero de pixeles blancos del lado derecho
        for i in range (0,50):
            for j in range (range_right_min,range_right_max):
                if (th2[i][j] == 255):
                    blanco_der += 1

        for i in range (150,200):
            for j in range (range_right_min,range_right_max):
                if (th2[i][j] == 255):
                    blanco_der += 1

        #print(th1.shape, "Shape th1")
        #print(th2.shape, "Shape th2")
        #print(blanco_der)
        #print(blanco_izq)
        pub_der.publish(blanco_der)
        pub_izq.publish(blanco_izq)
        #print(len(th2))

        if(blanco_der > blanco_izq):
            print("Girar Derecha")
            msg = "turn_right"
            pub.publish(msg)
        elif (blanco_der < blanco_izq):
            print("Girar Izquierda")
            msg = "turn_left"
            pub.publish(msg)
                    
        else:
            print("voy bien :)")
            msg = "forward"
            pub.publish(msg)

        th2 = cv2.line(th2, (range_left_min,0), (range_left_min,200), color = (255, 250, 255), thickness = 4)
        th2 = cv2.line(th2, (range_left_max,0), (range_left_max,200), color = (255, 250, 255), thickness = 4)
        th2 = cv2.line(th2, (range_right_min,0), (range_right_min,200), color = (255, 250, 255), thickness = 4)
        th2 = cv2.line(th2, (range_right_max,0), (range_right_max,200), color = (255, 250, 255), thickness = 4)
        cv2.imshow('ddd', th2)
        cv2.imshow('frame', frame)
        #cv2.imshow('Normal', frame)
        key = cv2.waitKey(20)

        if (key == 27):
            break

    video.release()
    cv2.destroyAllWIndows()
