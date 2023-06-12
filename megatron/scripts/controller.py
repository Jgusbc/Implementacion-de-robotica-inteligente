#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from line_detector.msg import values

kr_x = 0.0025
kl_x = 0.0025
kr_w = 0.005 
kl_w = 0.005

wr_x = 0.0
wr_w = 0.0
wl_x = 0.0
wl_w = 0.0

error_x = 0.0
error_w = 0.0

wr_final = 0.0
wl_final = 0.0

direction = "line_follower"
velocity = "normal_velocity"

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

def instruction_cb(msg):
    global direction, velocity
    direction = msg.direction
    velocity = msg.velocity

def line_follower():
    global error_w, error_x, wr_w, wl_w, wl_x, wr_x, wr_final, wl_final, kr_w, kr_x, kl_w, kl_x
    print("Error x = " + str(error_x))
    print("Error w = " + str(error_w))

    wr_x = 0.5 - kr_x * error_x
    wl_x = 0.5 + kl_x * error_x

    wr_w = - kr_w * error_w
    wl_w = kl_w * error_w

    wr_final = (wr_x + wr_w)/2
    wl_final = (wl_x + wl_w)/2

def turn_right():
    global wr_final, wl_final
    wr_final = 0.5
    wl_final = 0.8

def turn_left():
    global wr_final, wl_final
    wr_final = 0.8
    wl_final = 0.5

def forward():
    global wr_final, wl_final
    wr_final = 0.5
    wl_final = 0.5

if __name__ == '__main__':
    rospy.init_node("controller")
    rospy.loginfo("Node initialized")
    
    pwmL = rospy.Publisher("/cmd_pwmL", Float32, queue_size=10)
    pwmR = rospy.Publisher("/cmd_pwmR", Float32, queue_size=10)
    rospy.Subscriber("/error_x", Float32, cb_errorx)
    rospy.Subscriber("/error_w", Float32, cb_errorw)
    rospy.Subscriber("/instruction", values, instruction_cb)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():
        
        if(direction == "right"):
            turn_right()
        elif(direction == "left"):
            turn_left()
        elif(direction == "forward"):
            forward()
        elif(direction == "line_follower"):
            line_follower()

        if(velocity == "stop"):
            wl_final = 0
            wr_final = 0
        elif(velocity == "slow_down"):
            wl_final *= 0.7
            wr_final *= 0.7
        
        pwmL.publish(wl_final)
        pwmR.publish(wr_final*0.9)

        rate.sleep()


