#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from xarm_msgs.srv import *

fps = 10

kp_linx = 0.00001
kp_liny = 0.00001
kp_angz = 0.001
lower = np.array([170, 120, 100], dtype=np.uint8)
upper = np.array([179, 255, 255], dtype=np.uint8)
lower2 = np.array([0, 120, 100], dtype=np.uint8)
upper2 = np.array([10, 255, 255], dtype=np.uint8)
xarm_move_msg = MoveVelo()
xarm_move_msg.jnt_sync = 0
xarm_move_msg.coord = 1

def stop():
    cv2.destroyAllWindows()
    cap.release()


if __name__ == '__main__':
    cap = cv2.VideoCapture(2)

    rospy.wait_for_service('/xarm/velo_move_line')

    move_line = rospy.ServiceProxy('/xarm/velo_move_line', MoveVelo)
    motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
    set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
    set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
    get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
    home = rospy.ServiceProxy('/xarm/go_home', Move)

    rospy.init_node("xarm_visservo")
    rate = rospy.Rate(fps)
    rospy.on_shutdown(stop)

    try:
        motion_en(8, 1)
        set_mode(0)
        set_state(0)
        req = MoveRequest()  # MoveRequest for go_home
        req.mvvelo = 0.7
        req.mvacc = 3.5
        req.mvtime = 0
        home(req)

    except rospy.ServiceException as e:
        print("Before servo_cartesian, service call failed: %s" % e)
        exit(-1)

    set_mode(5)

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
        error = np.array([[320, 240],[320, 240],[320, 240],[320, 240]]) - box
        error = error.sum(axis=0)
        theta_error = np.arctan2((box[2][1]-box[1][1]), (box[2][0] - box[1][0]))
        theta_error = -np.arctan2(np.sin(theta_error), np.cos(theta_error))
        xarm_move_msg.velocities = [error[0], -error[1], 0, 0, 0, theta_error]
        move_line(xarm_move_msg)
        cv2.imshow("filter", gray)
        cv2.imshow("box", rot_bbox)
        # print(box.shape)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
        rate.sleep()
