#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image #CameraInfo
#Obtained by doing rostopic info on the topic(s) to extract
#rosmsg show sensor_msgs/Image,CameraInfo tells us the structure of the message
from message_filters import Subscriber, TimeSynchronizer
#Filters for a certain message type and invokes a callback function 
from cv_bridge import CvBridge
#Enables us to interface openCV with ROS
import cv2
import rosbag
import os

if not os.path.exists('./left'):
    os.mkdir('./left') 
if not os.path.exists('./right'):
    os.mkdir('./right') 
bridge = CvBridge()
left = 0
right = 0
bag = rosbag.Bag("./../toBag/kitti_2011_09_26_drive_0002_synced.bag")
try:
    for topic, msg, t in bag.read_messages(topics=["/kitti/camera_color_left/image_raw", "/kitti/camera_color_right/image_raw", "/tf"]):
        tf = 1
        if topic == "/kitti/camera_color_left/image_raw":
            img_name = str(left).zfill(3)
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite("./left/" + img_name + ".png", cv_img)
            left += 1
        elif topic == "/kitti/camera_color_right/image_raw":
            img_name = str(right).zfill(3)
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite("./right/" + img_name + ".png", cv_img)
            right += 1
except Exception as e:
    print("error boss")
bag.close()