#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

lineal = 0
angular = 0

r = 0.05
l = 0.19
vel_max = 13.65

def convert(vel):
    global lineal, angular

    lineal = vel.linear.x
    angular = vel.angular.z



if __name__ == '__main__':
    pwmL = rospy.Publisher("cmd_pwmL", Float32, queue_size=10)
    pwmR = rospy.Publisher("cmd_pwmR", Float32, queue_size=10)

    rospy.init_node("converter")

    cmd_vel = rospy.Subscriber("cmd_vel", Twist, convert)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        wl = (lineal/r) - ((angular*l)/(2*r))
        wr = (lineal/r) + ((angular*l)/(2*r))

        #wr = (1/(2*r))*(2*lineal + l*angular)
        #wl = (1/(2*r))*(2*lineal - l*angular)

        if (wr >= 13):
            wr = 13
        elif (wr <= -13):
            wr = -13

        if (wl > 13):
            wl = 13
        elif (wl < -13):
            wl = -13

        pwm_right = (wr/vel_max)
        pwm_left = (wl/vel_max)


        #rospy.loginfo("PWM R: %f", pwm_right)
        #rospy.loginfo("PWM L: %f", pwm_left)

        rospy.loginfo("wr: %f", wr)
        rospy.loginfo("wl: %f", wl)

        pwmL.publish(pwm_left)
        pwmR.publish(pwm_right)

        rate.sleep()
