#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
from collections import deque
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from see_maze.srv import ctr_pos
# from see_maze.srv import maze_bin

global flag_init
flag_init = True

def dist(a,b):
    return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5

def chOutlineOrder(approx):
    # print(approx)
    dist = [0,0,0,0]
    min_dist = 2000  # bigger than image height+width
    min_index = 0
    for i in range(4):
        dist[i] = sum(approx[i][0])
        if dist[i] < min_dist:
            min_dist = dist[i]
            min_index = i
    # print(min_index,min_dist)
    approx_ch = approx.copy()
    j = min_index
    for i in range(4):
        approx_ch[i] = approx[j]
        j += 1
        if j == 4:
            j = 0
    # print(approx_ch)
    return approx_ch

def getOutline(frame):
    # HSV light blue
    # color_min = np.array([105, 100, 200],np.uint8)
    # color_max = np.array([118, 180, 255],np.uint8)
    color_min = np.array([90, 100, 120],np.uint8)
    color_max = np.array([135, 255, 255],np.uint8)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color_min, color_max)
    res = cv2.bitwise_and(hsv, hsv, mask= mask)

    kernel1 = np.ones((5,5),np.float32)/25
    kernel2 = np.ones((3,3),np.float32)/9

    res = cv2.erode(res, kernel2, iterations=1)
    res = cv2.dilate(res, kernel2, iterations=1)
    # res = cv2.dilate(res, kernel2, iterations=1)
    canny = cv2.Canny(res,300,600)
    canny = cv2.dilate(canny, kernel2, iterations=1)

    _, contours, _ = cv2.findContours(canny,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) !=0:
        c = max(contours, key = cv2.contourArea)
        cnt = c
        M_perspective, approx = getTf(cnt)
        # if approx.size == 8:
        #     flag_validity = True
        # else:
        #     flag_validity = False
        return [M_perspective, approx, res ]

def getTf(cnt):
    epsilon = 0.05*cv2.arcLength(cnt,True)
    approx = cv2.approxPolyDP(cnt,epsilon,True)
    approx = chOutlineOrder(approx)
    ori_pos = np.float32([approx[0][0],approx[1][0],approx[2][0],approx[3][0]])
    dst_pos = np.float32([[0,0],[0,300],[300,300],[300,0]])
    M_perspective = cv2.getPerspectiveTransform(ori_pos,dst_pos)
    return [M_perspective,approx]

# def obj_identify(frame_in, obj):
#     # frame_in = cv2.resize(frame_in,(640,360))
#     frame = frame_in.copy()
#     rows, cols, channels = frame.shape

#     M_perspective, approx, res = getOutline(frame)  #### don't remove

def obj_identify_ball(frame_in):
    frame = frame_in.copy()
    rows, cols, channels = frame.shape

    M_perspective, approx, _ = getOutline(frame)

    # HSV ball
    color_min = np.array([0, 0, 170],np.uint8)
    color_max = np.array([100, 70, 255],np.uint8)
    # color_min = np.array([0, 30, 150],np.uint8)
    # color_max = np.array([35, 90, 255],np.uint8)

    frame_mask = frame.copy()

    # get rid of image area outside outline
    outline_min = np.array([255, 0, 0],np.uint8)
    outline_max = np.array([255, 0, 0],np.uint8)
    # fill in ROI with (255,0,0) BGR color
    cv2.drawContours(frame_mask, [approx], -1, (255, 0, 0),thickness=-1)
    mask_outline = cv2.inRange(frame_mask, outline_min, outline_max)
    # mask_inv = cv2.bitwise_not(mask_outline)
    maze_range = cv2.bitwise_and(frame, frame, mask= mask_outline)
    # cv2.imshow("maze_range",maze_range)
    # cv2.imshow('mask',mask_outline)

    hsv = cv2.cvtColor(maze_range, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color_min, color_max)
    res = cv2.bitwise_and(hsv, hsv, mask= mask)

    kernel1 = np.ones((5,5),np.float32)/25
    kernel2 = np.ones((3,3),np.float32)/9
    res = cv2.erode(res, kernel2, iterations=1)
    res = cv2.dilate(res, kernel2, iterations=1)

    # res = cv2.filter2D(res, -1, kernel)
    canny = cv2.Canny(res,300,600)
    # canny = cv2.erode(canny, kernel2, iterations=1)
    canny = cv2.dilate(canny, kernel2, iterations=1)

    _, contours, _ = cv2.findContours(canny,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) !=0:
        c = max(contours, key = cv2.contourArea)
        cnt = c

        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        radius = int(radius)

        # cv2.drawContours(frame, contours, -1, (0, 0, 255),3)
        cv2.circle(frame,center,radius,(0,0,255),2)

        center = np.array(center)
        center = np.append(center,1)
        center = np.reshape(center,(3,1))
        center_tf = np.dot(M_perspective, center )
        center_tf = center_tf/center_tf[2][0]
        center_tf = center_tf.astype(np.int32)
        center_tf_x = center_tf[0][0]
        center_tf_y = center_tf[1][0]
        center_tf = (center_tf_x,center_tf_y)

        # frame_backup = frame_in.copy()
        # img_perspective = obj_identify_maze(frame_backup)
        # cv2.circle(img_perspective,center_tf,11,150,-1)
        # cv2.circle(maze_bin_static,center_tf,11,150,-1)
        # cv2.imshow('img_perspective',img_perspective)
        # cv2.imshow('maze_normalized',maze_bin_static)
        # cv2.imshow("maze_range",maze_range)

        # cv2.drawContours(frame, [approx], -1, (255, 0, 0),3)
        # cv2.imshow('frame',frame)
        return [center_tf, approx]

def obj_identify_maze(frame_in):
    frame = frame_in.copy()
    rows, cols, channels = frame.shape

    M_perspective, approx, res = getOutline(frame)

    # HSV light blue
    # color_min = np.array([40, 30, 80],np.uint8)
    # color_max = np.array([125, 255, 255],np.uint8)

    # M_perspective, approx, res = getOutline(frame)

    img_perspective = cv2.warpPerspective(res, M_perspective, (cols, rows))
    img_perspective = np.copy(img_perspective[:300,:300])
    img_perspective = cv2.cvtColor(img_perspective, cv2.COLOR_HSV2BGR)
    img_perspective = cv2.cvtColor(img_perspective, cv2.COLOR_BGR2GRAY)
    _, img_perspective = cv2.threshold(img_perspective,50, 255,cv2.THRESH_BINARY)

    # cv2.drawContours(frame, [approx], -1, (255, 0, 0),3)
    # cv2.imshow('outline',frame)
    # cv2.imshow('img_perspective',img_perspective)
    return img_perspective

def obj_identify_sticker(frame_in):
    frame = frame_in.copy()
    rows, cols, channels = frame.shape

    M_perspective, approx, _ = getOutline(frame)
        # HSV green sticker
    color_min = np.array([35, 90, 50],np.uint8)
    color_max = np.array([85, 255, 255],np.uint8)

    frame_mask = frame.copy()
    # M_perspective, approx, _ = getOutline(frame)

    # get rid of image area outside outline
    outline_min = np.array([255, 0, 0],np.uint8)
    outline_max = np.array([255, 0, 0],np.uint8)
    # fill in ROI with (255,0,0) BGR color
    cv2.drawContours(frame_mask, [approx], -1, (255, 0, 0),thickness=-1)
    mask_outline = cv2.inRange(frame_mask, outline_min, outline_max)
    # mask_inv = cv2.bitwise_not(mask_outline)
    maze_range = cv2.bitwise_and(frame, frame, mask= mask_outline)
    # cv2.imshow("maze_range",maze_range)
    # cv2.imshow('mask',mask_outline)

    hsv = cv2.cvtColor(maze_range, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color_min, color_max)
    res = cv2.bitwise_and(hsv, hsv, mask= mask)

    kernel1 = np.ones((5,5),np.float32)/25
    kernel2 = np.ones((3,3),np.float32)/9
    res = cv2.erode(res, kernel2, iterations=1)
    res = cv2.dilate(res, kernel2, iterations=1)

    # res = cv2.filter2D(res, -1, kernel)
    canny = cv2.Canny(res,300,600)
    # canny = cv2.erode(canny, kernel2, iterations=1)
    canny = cv2.dilate(canny, kernel2, iterations=1)

    _, contours, _ = cv2.findContours(canny,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) !=0:
        c = max(contours, key = cv2.contourArea)
        cnt = c

        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        radius = int(radius)

        # cv2.drawContours(frame, contours, -1, (0, 0, 255),3)
        cv2.circle(frame,center,radius,(0,0,255),2)
        # cv2.imshow('framwe',frame)
        # cv2.waitKey(1)

        center = np.array(center)
        center = np.append(center,1)
        center = np.reshape(center,(3,1))
        center_tf = np.dot(M_perspective, center )
        center_tf = center_tf/center_tf[2][0]
        center_tf = center_tf.astype(np.int32)
        center_tf_x = center_tf[0][0]
        center_tf_y = center_tf[1][0]
        center_tf = (center_tf_x,center_tf_y)

        frame_backup = frame_in.copy()
        img_perspective = obj_identify_maze(frame_backup)
        cv2.line(img_perspective,(center_tf_x-8,center_tf_y-8),(center_tf_x+8,center_tf_y+8),150,5)
        cv2.line(img_perspective,(center_tf_x-8,center_tf_y+8),(center_tf_x+8,center_tf_y-8),150,5)
        # cv2.circle(img_perspective,center_tf,11,150,-1)
        # cv2.imshow('img_perspective',img_perspective)
        # cv2.imshow('frame',frame)
        # cv2.imshow("maze_range",maze_range)
        return center_tf

def dest(req):
    global device

    cap = cv2.VideoCapture(device)
    ret, frame_raw = cap.read()
    while (ret):
    # for tt in range(10):
        try:
            for i in range(5):
                ret, frame_raw = cap.read()
                cv2.waitKey(10)
            pos = obj_identify_sticker(frame_raw)
            pos_x = pos[0]
            pos_y = pos[1]
            pos_z = 0
            pos_ros = Point(pos_x,pos_y,0)
            print("hi")
            break
        except:
            print("Did not find the destination.")

    cap.release()
    # pos_ros = Point(0,0,0)
    return pos_ros

def main():
    global device

    rospy.init_node('maze_dest_server', anonymous=False)
    s = rospy.Service('maze_dest', ctr_pos, dest)
    device = rospy.get_param('~cam',default=0)

    # pub = rospy.Publisher("ball_pos", Point, queue_size= 10 )
    # bridge = CvBridge()
    # sub = rospy.Subscriber("/usb_cam/image_raw",Image, callback)

    # aaaa = cv2.imread("my_photo-14.jpg")
    # cv2.imshow("aaaa",aaaa)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

