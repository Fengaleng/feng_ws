#!/usr/bin/env python

__author__ =  'Mathieu Ashby <mathieu-ulysse.ashby@polymtl.ca>'
__version__=  '0.1'
__license__ = 'BSD'

import numpy as np
import cv2
import rospy
import roslib
from target_detection import detector
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import time
#import yaml

def callback(ros_data):
    global avg_position,avg_rot,avg_idx
    start_time = time.time()
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")

    detector_object = detector.Detector(cv_image)
    detector_object.run()
    cv2.imshow('cv_img', detector_object.img)
    cv2.waitKey(2)
    if detector_object.found_rect_flag and got_first_odom:

        rot_mat = np.zeros((3,3))
        cv2.Rodrigues(detector_object.rvec,rot_mat)
        # Go from camera to world coordinate
        R_trans = np.array([[0, 0, 1],[-1, 0, 0],[0, -1, 0]])

        detector_object.rvec = rot_mat[0:3,0]
        avg_position = (avg_position*avg_idx+odom_position + np.dot(odom_rot[0:3,0:3],np.dot(R_trans,detector_object.tvec)))/(avg_idx+1)
        # Get ROtation matrix of actual position with correct rotation matrixes(transform from object to cam (rodrigues), then transform form cam to world (R_trans), then multiply by odom)
        # Get x vector from rotation matrix and average
        R_base = np.array([[0, -1, 0],[0, 0, 1],[-1, 0,0]])
        rot_mat_total =np.dot(np.dot(odom_rot[0:3,0:3],np.dot(R_trans,rot_mat)),R_base)
        avg_rot = (np.dot(avg_rot,avg_idx)+rot_mat_total[0:3,0].reshape(3,1))/(avg_idx+1)
        avg_rot = avg_rot/np.linalg.norm(avg_rot)
        avg_idx = avg_idx+1
        #print("estimate")
        #print(odom_position + np.dot(odom_rot[0:3,0:3],np.dot(R_trans,detector_object.tvec)))
        #print(rot_mat_total[0:3,0].reshape(3,1))
        #print(avg_idx)

def OdomCallback(ros_data):
    global odom_position,odom_rot,got_first_odom
    #dom_position[0,0] = ros_data.pose.pose.position

    if not got_first_odom:
        got_first_odom = True

    odom_position[0,0] = ros_data.pose.pose.position.x
    odom_position[1,0] = ros_data.pose.pose.position.y
    odom_position[2,0] = ros_data.pose.pose.position.z
    odom_rot = quaternion_matrix([ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y, ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w])
    rot_vec = odom_rot[0:3,0]

if __name__ == '__main__':
    rospy.init_node('target_detector_node', anonymous=True)
    # detector = class

    avg_position = np.zeros((3,1))
    avg_position[0,0]= 2.41
    avg_position[1,0]= 5.5
    avg_position[2,0]= 2.81
    avg_rot = np.zeros((3,1))
    odom_position = np.zeros((3,1))
    odom_rot = np.zeros((3,3))
    avg_idx = 1
    got_first_odom = False

    print("Started target detection node")
    rospy.Subscriber("/firefly/camera_cam_test/image_raw", Image, callback, queue_size = 1)
    rospy.Subscriber("/firefly/msf_core/odometry", Odometry, OdomCallback, queue_size = 1)
    rospy.spin()
