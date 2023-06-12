#!/usr/bin/env python 

import rospy
from std_msgs.msg import String
from line_detector.msg import values

signal = None
color = None
msg = values()
msg.velocity = "normal_velocity"
msg.direction = "line_follower"

def signal_cb(msg):
    global signal
    signal = msg.data 

def color_cb(msg):
    global color
    color = msg.data 

if __name__ == '__main__':
    rospy.init_node('instructions')
    rospy.loginfo('Node initialized')
    rospy.Subscriber('signals', String, signal_cb)
    rospy.Subscriber('color', String, color_cb)
    pub = rospy.Publisher('instruction', values, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        if(color == "red" or signal == "stop"):
            msg.velocity = "stop"
        elif(color == "yellow" or signal == "construction"):
            msg.velocity = "slow_down"
        elif(color == "green"):
            msg.velocity = "normal_velocity"

        if(signal == "forward"):
            msg.direction = "forward"
        elif(signal == "right"):
            msg.direction = "right"
        elif(signal == "left"):
            msg.direction = "left"

        pub.publish(msg)

        signal = None
        color = None
        msg.velocity = "normal_velocity"
        msg.direction = "line_follower"
        rate.sleep()