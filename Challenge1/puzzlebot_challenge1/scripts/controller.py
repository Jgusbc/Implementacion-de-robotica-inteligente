#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist

def stop():
    msg.linear.x = 0
    msg.linear.z = 0
    cmd_vel.publish(msg)

msg = Twist()

if __name__ == '__main__':
    global cmd_vel
    cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    rospy.init_node("Controller")
    rate = rospy.Rate(10)

    vel_linear = 0.5
    vel_angular = 0.5
    msg.linear.x = vel_linear
    flag = 1
    flag2 = 0

    rospy.on_shutdown(stop)

    rate.sleep()
    tiempo_inicial = rospy.get_time()

    while not rospy.is_shutdown():
        tiempo = rospy.get_time() - tiempo_inicial

        rospy.loginfo("tiempo %s", tiempo)
        rospy.loginfo("tiempo inicial %s", tiempo_inicial)
        if(vel_linear > 0):
            if(tiempo >= 2/vel_linear  and flag == 1):
                vel_linear = 0
                msg.linear.x = 0
                msg.angular.z =vel_angular
                tiempo_inicial = rospy.get_time()
                flag = 0

        if(vel_angular>0):
            if(tiempo >= (np.pi/2)/vel_angular  and flag2 == 1):
                msg.angular.z = 0
                vel_linear = 0.5
                msg.linear.x = vel_linear
                tiempo_inicial = rospy.get_time()
                flag = 1
                flag2 = 0

        cmd_vel.publish(msg)

        if(flag == 0):
            flag2 = 1

        rate.sleep()
