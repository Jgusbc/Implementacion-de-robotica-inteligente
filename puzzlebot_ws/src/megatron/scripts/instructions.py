#!/usr/bin/env python 

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from megatron.msg import values
from std_msgs.msg import Bool

signal = "no_signal"
color = "no_color"
msg = values()
msg.velocity = "normal_velocity"
msg.direction = "line_follower"
crosswalk = False

def signal_cb(msg):
    global signal
    signal = msg.data 

def color_cb(msg):
    global color
    color = msg.data 

def crosswalk_cb(msg):
    global crosswalk
    crosswalk = msg.data

if __name__ == '__main__':
    rospy.init_node('instructions')
    rospy.loginfo('Node initialized')
    rospy.Subscriber('signal', Int32, signal_cb)
    rospy.Subscriber('color', String, color_cb)
    rospy.Subscriber('crosswalk', Bool, crosswalk_cb)
    pub = rospy.Publisher('instruction', values, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        if(color == "red" or signal == 4):
            msg.velocity = "stop"
        elif(color == "yellow" or signal == 5):
            msg.velocity = "slow_down"
        elif(color == "green" or color == "no_color"):
            msg.velocity = "normal_velocity"

        if(signal == 0 and crosswalk):
            msg.direction = "forward"
        elif(signal == 2 and crosswalk):
            msg.direction = "right"
        elif(signal == 6 and crosswalk):
            msg.direction = "left"
        elif(signal == 404 or signal == 202):
            msg.direction = "line_follower"

        pub.publish(msg)

        
        msg.velocity = "normal_velocity"
        msg.direction = "line_follower"
        rate.sleep()