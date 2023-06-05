#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32

direction = ""
left_wheel = 0.0
right_wheel = 0.0
blanco_der = 0.0
blanco_izq = 0.0

def forward():
    global left_wheel, right_wheel
    print("forward")
    left_wheel = 0.22
    right_wheel = 0.22
    pwmL.publish(left_wheel)
    pwmR.publish(right_wheel)
    #print("Left wheel: ", left_wheel)
    #print("Right wheel: ", right_wheel)
    
def turn_left():
    global left_wheel, right_wheel, blanco_izq
    print("turn left")
    left_wheel = 0.2 - 0.7*(blanco_izq/1250)
    right_wheel =  1.3 * (blanco_izq/1250) + 0.2
    pwmL.publish(left_wheel)
    pwmR.publish(right_wheel)
    #print("Left wheel: ", left_wheel)
    #print("Right wheel: ", right_wheel)

def turn_right():
    global left_wheel, right_wheel, blanco_der
    print("turn right")
    left_wheel = 1.3 * (blanco_der/1250) + 0.2
    right_wheel = 0.2 - 0.7*(blanco_der/1250)
    pwmL.publish(left_wheel)
    pwmR.publish(right_wheel)
    #print("Left wheel: ", left_wheel)
    #print("Right wheel: ", right_wheel)

def stop():
    global left_wheel, right_wheel
    print("stop")
    left_wheel = 0.0
    right_wheel = 0.0
    pwmL.publish(left_wheel)
    pwmR.publish(right_wheel)

def cb_direction(msg):
    global direction
    direction = msg.data

def cb_blanco_der(msg):
    global blanco_der
    blanco_der = msg.data

def cb_blanco_izq(msg):
    global blanco_izq
    blanco_izq = msg.data

if __name__ == '__main__':
    rospy.init_node("converter")
    rospy.loginfo("Node initialized")
    pwmL = rospy.Publisher("/cmd_pwmL", Float32, queue_size=10)
    pwmR = rospy.Publisher("/cmd_pwmR", Float32, queue_size=10)
    rospy.Subscriber("/direction", String, cb_direction)
    rospy.Subscriber("/blanco_der", Float32, cb_blanco_der)
    rospy.Subscriber("/blanco_izq", Float32, cb_blanco_izq)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():

        print(direction)
        if direction == "forward":
            forward()
        elif direction == "turn_left":
            turn_left()
        elif direction == "turn_right":
            turn_right()
        else:
            stop()

        rate.sleep()


