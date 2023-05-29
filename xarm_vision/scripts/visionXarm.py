#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import cv2

fps = 10

xarm_error = Twist()
kp_linx = 0.00001
kp_liny = 0.00001
kp_angz = 0.001
cap = cv2.VideoCapture(0)
lower = np.array([178, 120, 130], dtype=np.uint8)
upper = np.array([179, 255, 255], dtype=np.uint8)
lower2 = np.array([0, 120, 130], dtype=np.uint8)
upper2 = np.array([20, 255, 255], dtype=np.uint8)


def stop():
    cv2.destroyAllWindows()
    cap.release()

rospy.init_node("xarm_vision")
pub = rospy.Publisher('/u_xarm', Twist, queue_size=10)
rate = rospy.Rate(fps)
rospy.on_shutdown(stop)

# Initiate ORB detector
while not rospy.is_shutdown():
    ret, img = cap.read()
    if not ret:
        break
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower, upper)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = mask1 + mask2
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img2 = cv2.bitwise_and(img, img, mask=mask)
    gray = 255-img2
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 75, 2)
    thresh = 255 - thresh

    # apply morphology
    kernel = np.ones((5, 5), np.uint8)
    rect = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    rect = cv2.morphologyEx(rect, cv2.MORPH_CLOSE, kernel)

    # thin
    kernel = np.ones((5, 5), np.uint8)
    rect = cv2.morphologyEx(rect, cv2.MORPH_ERODE, kernel)

    # get largest contour
    contours = cv2.findContours(rect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    for c in contours:
        area_thresh = 0
        area = cv2.contourArea(c)
        if area > area_thresh:
            area = area_thresh
            big_contour = c

    # get rotated rectangle from contour
    rot_rect = cv2.minAreaRect(big_contour)
    box = cv2.boxPoints(rot_rect)
    box = np.int0(box)
    if box[0][0] > box[2][0]:
        box = np.array([box[1], box[2], box[3], box[0]])
    rot_bbox = img.copy()
    cv2.drawContours(rot_bbox, [box], 0, (0, 0, 255), 2)
    rot_bbox = cv2.circle(rot_bbox, (box[0][0], box[0][1]), 3, (255,255,255), 1)
    rot_bbox = cv2.circle(rot_bbox, (box[1][0], box[1][1]), 3, (255,255,255), 3)
    rot_bbox = cv2.circle(rot_bbox, (box[2][0], box[2][1]), 3, (255,255,255), 5)
    rot_bbox = cv2.circle(rot_bbox, (box[3][0], box[3][1]), 3, (255,255,255), 7)
    print(type(box[0]))
    error = np.array([[320, 240],[320, 240],[320, 240],[320, 240]]) - box
    error = error.sum(axis=0)
    theta_error = np.arctan2((box[2][1]-box[1][1]), (box[2][0] - box[1][0]))
    theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
    print(error)
    print(theta_error)
    xarm_error.linear.x = error[0] * kp_linx
    xarm_error.linear.y = error[1] * kp_liny
    xarm_error.angular.z = -theta_error * kp_angz
    pub.publish(xarm_error)
    cv2.imshow("filter", gray)
    cv2.imshow("box", rot_bbox)
    # print(box.shape)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
    rate.sleep()
