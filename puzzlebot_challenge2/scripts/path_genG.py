#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

class Generator:
    def __init__(self):
        self.pub = rospy.Publisher("setPoint", Pose2D, queue_size=10)
        self.sub = rospy.Subscriber("route_end", Float32, self.callback)
        self.setPoint = Pose2D()
        self.received = 0

    def callback(self, msg):
        rospy.loginfo("done")
        x = input("X pos: ")
        y = input("Y pos: ")
        gen.generate(x, y)
        self.received = msg.data

    def generate(self, x, y):
        self.setPoint.x = x
        self.setPoint.y = y
        self.setPoint.theta = 0
        self.pub.publish(self.setPoint)

if __name__ == "__main__":
    gen = Generator()
    rospy.init_node("generator")
    x = input("X pos: ")
    y = input("Y pos: ")
    gen.generate(x,y)
    rospy.spin()

