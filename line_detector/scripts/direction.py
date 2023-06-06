#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32

kr_x = 0.00048
kl_x = 0.00058
kr_w = 0.0035
kl_w = 0.003

wr_x = 0.0
wr_w = 0.0
wl_x = 0.0
wl_w = 0.0

error_x = 0.0
error_w = 0.0

wr_final = 0.0
wl_final = 0.0

def stop():
    global left_wheel, right_wheel
    print("stop")
    left_wheel = 0.0
    right_wheel = 0.0
    pwmL.publish(left_wheel)
    pwmR.publish(right_wheel)

def cb_errorx(msg):
    global error_x
    error_x = msg.data

def cb_errorw(msg):
    global error_w
    error_w = msg.data

if __name__ == '__main__':
    rospy.init_node("converter")
    rospy.loginfo("Node initialized")
    pwmL = rospy.Publisher("/cmd_pwmL", Float32, queue_size=10)
    pwmR = rospy.Publisher("/cmd_pwmR", Float32, queue_size=10)
    rospy.Subscriber("/error_x", Float32, cb_errorx)
    rospy.Subscriber("/error_w", Float32, cb_errorw)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():
        
        print("Error x = " + str(error_x))
        print("Error w = " + str(error_w))

        wr_x = 0.33 - kr_x * error_x
        wl_x = 0.33 + kl_x * error_x

        wr_w = 0.2 - kr_w * error_w
        wl_w = 0.2 + kl_w * error_w

        wr_final = (wr_x + wr_w)/2
        wl_final = (wl_x + wl_w)/2

        pwmL.publish(wl_final)
        pwmR.publish(wr_final)

        rate.sleep()


