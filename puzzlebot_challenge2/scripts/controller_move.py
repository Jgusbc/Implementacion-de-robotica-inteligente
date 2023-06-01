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
    print("Si entre al callback")
    msg_recibido = True

def angle_delimit(theta):
    return np.arctan2(np.sin(theta),np.cos(theta))

def controlador():
    global angulo, theta_angle, angular_kw, msg, xr, msg_recibido
    w_error = angle_delimit(np.arctan2(yr - 540, xr - 480) - theta_angle)
    #l_error = 480 - xr
    print("El error es: " + str(w_error))
    print("yr = " + str(yr))
    print("xr = " + str(xr))
    #if(msg_recibido):
    if(w_error > 0.1 or w_error < -0.1):
        u_w = angular_kw * w_error
        msg.angular.z = u_w
        msg.linear.x = 0.05
    else:
        msg.angular.z = 0
        msg.linear.x = 0.1
    #msg_recibido = False
    #else:
        #msg.angular.z = 0
        #msg.linear.x = 0

    
    
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
