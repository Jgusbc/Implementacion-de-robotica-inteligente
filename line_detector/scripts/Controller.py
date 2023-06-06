#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np
from geometry_msgs.msg import Twist
from line_detector.msg import values

angulo = 0.0
xr = 0.0
yr = 0.0
msg_recibido = False
msg = Twist()
theta_angle = np.pi/2
wheel_radio = 0.05
axle_track = 0.19
max_w = 8
wl = 0
wr = 0
dt = 1/100
angular_kw = 2


def wl_callback(msg):
    global wl
    wl = msg.data

def wr_callback(msg):
    global wr
    wr = msg.data

def callback(msg):
    global angulo, xr, yr, msg_recibido
    angulo = msg.angulo
    xr = msg.xr
    yr = msg.yr
    #rospy.loginfo("Si entre al callback")
    msg_recibido = True

def angle_delimit(theta):
    return np.arctan2(np.sin(theta),np.cos(theta))

def controlador():
    global angulo, theta_angle, angular_kw, msg, xr, msg_recibido, yr
    #w_error = angle_delimit(np.arctan2(yr - 540, xr - 480) - theta_angle)
    #l_error = 480 - xr
    #print("El error es: " + str(w_error))
    center_point = 510
    #print("yr = " + str(yr))
    #print("xr = " + str(xr))
    
    error_angle =angle_delimit( (np.pi/2) - angulo)
    #print ("angulo: " + str(angulo))
    #print("error de angulo: " + str(error_angle))

    if(msg_recibido):
        # ajuste por medio del angulo
        if(error_angle > 0.1):
            # girar manecillas del reloj, derecha
            print("Girar derecha")
            u_w = error_angle
            k_w = 0.1
            msg.angular.z = k_w * u_w 
            msg.linear.x = 0.8
        elif(error_angle < -0.1):
            # girar contramanecillas del reloj, izquierda
            print("Girar izquierda")
            u_w = error_angle
            k_w = -0.1
            msg.angular.z = k_w * u_w 
            msg.linear.x = 0.8

        elif(xr < (center_point - 15) and (yr > 360)):
            u_w = 0.4
            msg.angular.z = u_w
            msg.linear.x = 0.8
        elif(xr > (center_point + 15) and (yr > 360)):
            u_w = -0.4
            msg.angular.z = u_w
            msg.linear.x = 0.8
        else:
            msg.angular.z = 0
            msg.linear.x = 0.8
        msg_recibido = False
    else:
        msg.angular.z = 0
        msg.linear.x = 0

def shutdown():
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)

# Ruta del video a procesar
if __name__ == '__main__':
    rospy.init_node("Controller")
    rospy.loginfo("Node initiated")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    rospy.Subscriber("/line", values, callback)
    rospy.Subscriber("/wl", Float32, wl_callback)
    rospy.Subscriber("/wr", Float32, wr_callback)
    rate = rospy.Rate(100)
    #global msg 
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        controlador()
        pub.publish(msg)
        rate.sleep()
