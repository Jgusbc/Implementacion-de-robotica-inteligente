#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

lineal = 0
angular = 0
wr = 0
wl = 0

r = 0.05
l = 0.19
vel_max = 13.65

kr = 1.0
kl = 1.0

def callback_cmd_vel(msg):
    global lineal, angular
    lineal = msg.linear.x
    angular = msg.angular.z

def callback_wr(msg):
    global wr 
    wr = -1 * msg.data

def callback_wl(msg):
    global wl
    wl = msg.data

if __name__ == '__main__':

    rospy.init_node("wheel_controller")
    rospy.loginfo("Node Initialize")
    rate = rospy.Rate(10)

    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("/wr", Float32, callback_wr)
    rospy.Subscriber("/wr", Float32, callback_wl)

    pwmL = rospy.Publisher("cmd_pwmL", Float32, queue_size=10)
    pwmR = rospy.Publisher("cmd_pwmR", Float32, queue_size=10)

    while not rospy.is_shutdown():

        v_angular = (r * (wl - wr))/(2 * l)
        v_lineal = (r * (wl + wr)) / 2

        error_lineal = lineal - v_lineal
        error_angular = angular - v_angular

        w_left = (error_lineal/r) - ((error_angular*l)/(2*r))
        w_right = (error_lineal/r) + ((error_angular*l)/(2*r))*3

        pwm_right = kr * (w_right/vel_max)
        pwm_left = kl * (w_left/vel_max)

         #Limite maximo
        if(pwm_left > 1):
            pwm_left = 1
        elif(pwm_left < -1):
            pwm_left = -1
        if(pwm_right > 1):
            pwm_right = 1
        elif(pwm_right < -1):
            pwm_right = -1

        #Limite minimo
        if(pwm_left > 0 and pwm_left < 0.15):
            pwm_left = 0.15
        elif(pwm_left < 0 and pwm_left > -0.15):
            pwm_left = -0.15
        if(pwm_right > 0 and pwm_right < 0.15):
            pwm_right = 0.15
        elif(pwm_right < 0 and pwm_right > -0.15):
            pwm_right = -0.15

        if (lineal == 0 and angular == 0):
            pwm_right = 0
            pwm_left = 0

        print("pwm left: ", pwm_left)
        print("pwm right: ", pwm_right)
        rospy.loginfo("wr: %f", wr)
        rospy.loginfo("wl: %f", wl)

        pwmL.publish(pwm_left)
        pwmR.publish(pwm_right)

        rate.sleep()
