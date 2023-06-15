#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from megatron.msg import values
import time

kr_x = 0.0025
kl_x = 0.0025
kr_w = 0.003 
kl_w = 0.003

wr_x = 0.0
wr_w = 0.0
wl_x = 0.0
wl_w = 0.0

error_x = 0.0
error_w = 0.0

wr_final = 0.0
wl_final = 0.0

k_v = 1

direction = "line_follower"
velocity = "normal_velocity"

time_delay = 0 
time_delay_ini = 0


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
    global error_w, error_x, wr_w, wl_w, wl_x, wr_x, wr_final, wl_final, kr_w, kr_x, kl_w, kl_x, k_v
    print("Error x = " + str(error_x))
    print("Error w = " + str(error_w))

    wr_x = 0.37 - kr_x * error_x
    wl_x = 0.37 + kl_x * error_x

    wr_w = - kr_w * error_w
    wl_w = kl_w * error_w

    wr_final = (wr_x + wr_w)/2
    wl_final = (wl_x + wl_w)/2

    wr_final *=k_v
    wl_final *=k_v

    pwmL.publish(wl_final)
    pwmR.publish(wr_final*0.9)

def turn_right():
    global wr_final, wl_final, k_v, time_delay

    wr_final = 0.3*k_v
    wl_final = 0.3*k_v

    pwmL.publish(wl_final)
    pwmR.publish(wr_final*0.9)

    rospy.sleep(time_delay)

    wr_final = 0.22
    wl_final = 0.5

    pwmL.publish(wl_final)
    pwmR.publish(wr_final*0.9)

    rospy.sleep(time_delay-0.5)

def turn_left():
    global wr_final, wl_final, k_v, time_delay

    wr_final = 0.3*k_v
    wl_final = 0.3*k_v

    pwmL.publish(wl_final)
    pwmR.publish(wr_final*0.9)

    rospy.sleep(time_delay)

    wr_final = 0.5
    wl_final = 0.22

    pwmL.publish(wl_final)
    pwmR.publish(wr_final*0.9)

    rospy.sleep(time_delay-0.5)

def forward():
    global wr_final, wl_final
    wr_final = 0.5
    wl_final = 0.5

    pwmL.publish(wl_final)
    pwmR.publish(wr_final*0.9)

    rospy.sleep(time_delay)

if __name__ == '__main__':
    rospy.init_node("controller")
    rospy.loginfo("Node initialized")
    
    pwmL = rospy.Publisher("/cmd_pwmL", Float32, queue_size=1)
    pwmR = rospy.Publisher("/cmd_pwmR", Float32, queue_size=1)
    rospy.Subscriber("/error_x", Float32, cb_errorx)
    rospy.Subscriber("/error_w", Float32, cb_errorw)
    rospy.Subscriber("/instruction", values, instruction_cb)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():

        if(velocity == "stop"):
            k_v = 0

        elif(velocity == "slow_down"):
            k_v = 0.9
            time_delay = 2
        else:
            k_v = 1
            time_delay = 2
            
        if(direction == "right"):
            turn_right()
        elif(direction == "left"):
            turn_left()
    
        elif(direction == "forward"):
            forward()
            time_delay = 10

        elif(direction == "line_follower"):
            line_follower()
            time_delay = 0

        #time.sleep(time_delay_ini)
        

        #time.sleep(time_delay)

        rate.sleep()