#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
import numpy as np
from cv_bridge import CvBridge
from line_detector.msg import values

msg = values()

def draw_lines(img, lines, color=[255, 0, 0], thickness=7):
    global msg
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)
            msg.angulo = np.arctan2((y2-y1),(x2-x1))
            xr = (x1 + x2)/2
            yr = (y1 + y2)/2
            print("xr: " +str(xr))
            print("yr: " +str(yr))
            msg.xr = xr
            msg.yr = yr
            cv2.circle(img, (xr, yr), 2, (0,255,255),2)
            pub.publish(msg)

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
    #global msg
    rospy.init_node("line")
    pub = rospy.Publisher("/line", values, queue_size = 1)
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        #print("estoy en el while")
    # Realizar el seguimiento de leas en el video
        while True:
            ret, frame = cap.read()
            #print(ret)
            if not ret:
                break

            #pasar a grayscale
            grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            kernel_size = 9
            blur = cv2.GaussianBlur(grayscale, (kernel_size, kernel_size), 0)

            low_t = 30
            high_t = 50
            ignore_mask_color = 255
            edges = cv2.Canny(blur, low_t, high_t)

            vertices = np.array([[(80, frame.shape[0]), (350, 350), (650, 350), (frame.shape[1]-80, frame.shape[0])]], dtype=np.int32)
            mask = np.zeros_like(edges)
            cv2.fillPoly(mask, vertices, ignore_mask_color)
            #cv2.imshow("polly", polly)

            masked_edges = cv2.bitwise_and(edges, mask)
            masked_image = cv2.bitwise_and(frame, frame, mask=mask)

            # filtrar hsv
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            luz_baja = 0  # Valor s oscuras
            luz_alta = 100  # Valoreas oscuras
            umbral_bajo = np.array([0, 0, 0])
            umbral_alto = np.array([179, 255, 140])
            mascara = cv2.inRange(hsv, umbral_bajo, umbral_alto)

            resultado = cv2.bitwise_and(masked_edges, mascara)

            rho = 3
            theta = np.pi / 180
            threshold = 20
            min_line_len = 90
            max_line_gap = 20
            lines = cv2.HoughLinesP(resultado, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

            lineas_filtradas = []
            prom_lineas_filtradas = []

            print(type(lines))
            if lines is not None:

                x1_prom = 0
                x2_prom = 0
                y1_prom = 0
                y2_prom = 0
                for linea in lines:
                    x1, y1, x2, y2 = linea[0]
                    #print("x1: " +str(x1))
                    #print("y1: " +str(y1))
                    #print("x2: " +str(x2))
                    #print("y2: " +str(y2))
                    pendiente = (y2 - y1) / (x2 - x1 + 1e-6)
                    if abs(pendiente) > 0.1:
                        #print(pendiente)
                        lineas_filtradas.append([[x1,y1,x2,y2]])
                        
                        x1_prom += x1 
                        x2_prom += x2
                        y1_prom += y1 
                        y2_prom += y2

                if len(lineas_filtradas) > 0:
                    x1_prom = x1_prom / (len(lineas_filtradas)) 
                    x2_prom = x2_prom / (len(lineas_filtradas))
                    y1_prom = y1_prom / (len(lineas_filtradas))
                    y2_prom = y2_prom / (len(lineas_filtradas))
                
                
                prom_lineas_filtradas.append([[x1_prom, y1_prom, x2_prom, y2_prom]])

            #print("lines:")
            #print(lines)
            #print("/n Lineas filtradas")
            #print(lineas_filtradas)
            
            cv2.circle(frame, (510, 430), 2, (0,0,255),2)
            try:
                draw_lines(frame, prom_lineas_filtradas)
            except:
                print("no hay lineas")
            cv2.imshow('frame', frame)
            cv2.imshow('edges', edges)
            cv2.imshow('mascara', mascara)
            cv2.imshow('masacara edges', masked_edges)
            cv2.imshow('resultado', resultado)

            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        rate.sleep()