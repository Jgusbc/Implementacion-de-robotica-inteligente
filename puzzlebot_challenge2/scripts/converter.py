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
vel_max = 12.6

def convert(vel):
    global lineal, angular

    lineal = vel.linear.x
    angular = vel.angular.z



if __name__ == '__main__':
    global r, l, lineal, angular, vel_max
    pwmL = rospy.Publisher("cmd_pwmL", Float32, queue_size=10)
    pwmR = rospy.Publisher("cmd_pwmR", Float32, queue_size=10)

    rospy.init_node("converter")

    cmd_vel = rospy.Subscriber("cmd_vel", Twist, convert)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        wr = (1/(2*r))*(2*lineal + l*angular)
        wl = (1/(2*r))*(2*lineal - l*angular)

        pwm_right = wr/vel_max
        pwm_left = wl/vel_max

        rospy.loginfo("PWM R: %f", pwm_right)
        rospy.loginfo("PWM L: %f", pwm_left)
        pwmL.publish(pwm_left)
        pwmR.publish(pwm_right)

        rate.sleep()
