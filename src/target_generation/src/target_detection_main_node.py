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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from target_detection import trajectory_generation
from target_detection.msg import target_dect_msg
import time
import sys
#import yaml

class track_trajectory_gen:
    def __init__(self, targets_3d,targets_3d_orientation,showimage,threshold_distance_target,avg_spd,deg,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,targets_3d_ref_dist,start_threshhold,mode):
        print("Started target detection node")

        self.start_threshhold = start_threshhold
        self.targets_3d_ref_dist = targets_3d_ref_dist
        self.odom_position = np.zeros((3,1))
        self.odom_rot = np.zeros((3,3))
        self.numtargets = targets_3d.shape[0]
        self.targets = np.zeros((self.numtargets,5))
        self.target_orientation = np.zeros((self.numtargets,3))
        self.targets[0:self.numtargets+1,0:3] = targets_3d
        self.targets[0:self.numtargets+1,4:5] = np.ones((targets_3d.shape[0],1))
        self.target_orientation[0:self.numtargets+1,0:3] = targets_3d_orientation.copy()
        self.ref_target = np.zeros((self.numtargets,3))
        self.ref_target = targets_3d
        # todo pose of target tracker (only x axis direction)
        self.initial_position = np.zeros((3,1))
        self.showimage = showimage
        self.active_target = 0
        self.threshold_distance_target = threshold_distance_target
        #avg_idx = 1
        self.got_first_odom = False
        self.new_segment = False
        self.active_segment = 0
        self.deg = deg
        self.num_coeffs = self.deg*2+1+1
        self.threshold_dist_ref_recalculate = 0.25 #0.4
        self.time_optim_flag = False
        self.coefficients_x = np.zeros((1,self.num_coeffs))
        self.coefficients_y = np.zeros((1,self.num_coeffs))
        self.coefficients_z = np.zeros((1,self.num_coeffs))
        rate = 100.0
        self.rate_calc = 50.0;


        rospy.Timer(rospy.Duration(1/rate),self.Traj_manager)
        rospy.Timer(rospy.Duration(1/self.rate_calc),self.Traj_generator)



        self.step = 1/rate
        self.polytimer = 0.0
        self.avg_spd = avg_spd

        self.corr_const_flag = corr_const_flag
        self.corridor_radius = corridor_radius
        self.num_points_corr = num_points_corr
        self.corr_length = corr_length
        self.time_corr = time_corr
        self.traj_time_seg = np.array([0.0])
        self.segment_tracker = 0
        self.new_segment_result = False

        self.start_time = time.time()
        self.started_flag = False
        self.got_first_odom_flag = False

        self.last_x_val = 0
        self.last_y_val = 0
        self.last_z_val = 0
        self.last_x_vel_val = 0
        self.last_y_vel_val = 0
        self.last_z_vel_val = 0
        self.last_x_acc_val = 0
        self.last_y_acc_val = 0
        self.last_z_acc_val = 0
        self.first_start = False


        self.image_sub = rospy.Subscriber("/firefly/camera_cam_test/image_raw", Image, self.Imagecallback, queue_size = 1)
        self.odom_sub = rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, self.OdomCallback, queue_size = 1)
        self.imu_sub = rospy.Subscriber("/firefly/imu", Imu, self.Check_collision, queue_size = 1)
        self.ground_truth_sub = rospy.Subscriber("/firefly/ground_truth/odometry", Odometry, self.Get_groundtruth, queue_size = 1)
        self.traj_pub = rospy.Publisher("position", Vector3, queue_size=1)
        self.traj_pub_complete = rospy.Publisher("full_control_state",target_dect_msg, queue_size=1)





        self.coefficients_x_calc = np.zeros((1,self.num_coeffs))
        self.coefficients_y_calc = np.zeros((1,self.num_coeffs))
        self.coefficients_z_calc = np.zeros((1,self.num_coeffs))

        self.gt_orientation = np.array([1,0,0,0])


        print("Number of targets:")
        print(self.numtargets)

        self.mode = mode
        self.collide_state = 0

    def Check_collision(self,ros_data):

        #print(self.gt_orientation)
        orientation_matrix = quaternion_matrix([self.gt_orientation[1], self.gt_orientation[2], self.gt_orientation[3], self.gt_orientation[0]])
        orientation_matrix_int = orientation_matrix[0:3,0:3]
        acceleration_imu = np.array([[ros_data.linear_acceleration.x],[ros_data.linear_acceleration.y],[ros_data.linear_acceleration.z]])
        #print("imu")
        #print(acceleration_imu)
        corrected_imu = np.dot(orientation_matrix_int.transpose(),acceleration_imu)-np.array([[0],[0],[9.81]])
        #print(orientation_matrix.shape)
        #print(orientation_matrix)
        norm_imu = np.linalg.norm(corrected_imu)
        #print(corrected_imu)
        #print(norm_imu)
        delta_end = np.dot(self.target_orientation[-1,0:3],self.corr_length).reshape(3,1)
        distance_to_end = np.linalg.norm(delta_end+np.array([[self.targets[-1,0]-self.odom_position[0,0]],[self.targets[-1,1]-self.odom_position[1,0]],[self.targets[-1,2]-self.odom_position[2,0]]]))
        #print(distance_to_end)
        if (norm_imu > 40 and (time.time()-self.start_time)>20) or (time.time()-self.start_time)>300 :

            #print(acceleration_imu)

            print("The drone crashed or timeout")
            #print(norm_imu)
            #print(corrected_imu)
            #print(distance_to_end)
            self.collide_state = 1
            rospy.signal_shutdown("the drone collided")

        elif  distance_to_end < self.corr_length/2:
            rospy.signal_shutdown("Success")


    def Get_groundtruth(self,ros_data):
        self.gt_orientation[0] = ros_data.pose.pose.orientation.w
        self.gt_orientation[1] = ros_data.pose.pose.orientation.x
        self.gt_orientation[2] = ros_data.pose.pose.orientation.y
        self.gt_orientation[3] = ros_data.pose.pose.orientation.z

    def Traj_manager(self,ros_data):
        time_start = time.time()
        self.polytimer = self.polytimer+self.step
        cond_a = self.active_segment < self.numtargets
        cond_b_int = (self.segment_tracker) == np.size(self.traj_time_seg,0)-1
        cond_b_int_2 = self.polytimer+self.step >= self.traj_time_seg[np.size(self.traj_time_seg,0)-1]
        cond_b = cond_b_int and cond_b_int_2

        if (cond_a and not cond_b) or (not cond_a and cond_b):
            #time =
            if self.new_segment_result != 0:
                self.segment_tracker = 0
                self.polytimer = 0
                self.new_segment_result = False
            if np.size(self.traj_time_seg,0)>1 and self.traj_time_seg[self.segment_tracker]<self.polytimer and (self.segment_tracker+1)<np.size(self.traj_time_seg,0):
                print("swap")
                print(rospy.get_time())
                self.segment_tracker = self.segment_tracker+1
                self.polytimer = 0

            coeffs_x = np.flip(self.coefficients_x[self.segment_tracker*self.num_coeffs:self.segment_tracker*self.num_coeffs+self.num_coeffs]).copy()
            coeffs_y = np.flip(self.coefficients_y[self.segment_tracker*self.num_coeffs:self.segment_tracker*self.num_coeffs+self.num_coeffs]).copy()
            coeffs_z = np.flip(self.coefficients_z[self.segment_tracker*self.num_coeffs:self.segment_tracker*self.num_coeffs+self.num_coeffs]).copy()


            x_val = np.polyval(np.poly1d(coeffs_x.reshape(self.num_coeffs,)),self.polytimer)
            y_val = np.polyval(np.poly1d(coeffs_y.reshape(self.num_coeffs,)),self.polytimer)
            z_val = np.polyval(np.poly1d(coeffs_z.reshape(self.num_coeffs,)),self.polytimer)

            if self.traj_time_seg[self.segment_tracker]+1/self.rate_calc>self.polytimer and (self.segment_tracker+1)>np.size(self.traj_time_seg,0):
                segment_tracker = self.segment_tracker+1
                polytimer = 0
            else:
                segment_tracker = self.segment_tracker

            self.coefficients_x_calc = np.flip(self.coefficients_x[segment_tracker*self.num_coeffs:segment_tracker*self.num_coeffs+self.num_coeffs]).copy()
            self.coefficients_y_calc = np.flip(self.coefficients_y[segment_tracker*self.num_coeffs:segment_tracker*self.num_coeffs+self.num_coeffs]).copy()
            self.coefficients_z_calc = np.flip(self.coefficients_z[segment_tracker*self.num_coeffs:segment_tracker*self.num_coeffs+self.num_coeffs]).copy()


            x_dot_val = np.polyval(np.polyder(np.poly1d(coeffs_x.reshape(self.num_coeffs,)),1),self.polytimer)
            y_dot_val = np.polyval(np.polyder(np.poly1d(coeffs_y.reshape(self.num_coeffs,)),1),self.polytimer)
            z_dot_val = np.polyval(np.polyder(np.poly1d(coeffs_z.reshape(self.num_coeffs,)),1),self.polytimer)


            x_dot_dot_val = np.polyval(np.polyder(np.poly1d(coeffs_x.reshape(self.num_coeffs,)),2),self.polytimer)
            y_dot_dot_val = np.polyval(np.polyder(np.poly1d(coeffs_y.reshape(self.num_coeffs,)),2),self.polytimer)
            z_dot_dot_val = np.polyval(np.polyder(np.poly1d(coeffs_z.reshape(self.num_coeffs,)),2),self.polytimer)

            x_jerk_val = np.polyval(np.polyder(np.poly1d(coeffs_x.reshape(self.num_coeffs,)),3),self.polytimer)
            y_jerk_val = np.polyval(np.polyder(np.poly1d(coeffs_y.reshape(self.num_coeffs,)),3),self.polytimer)
            z_jerk_val = np.polyval(np.polyder(np.poly1d(coeffs_z.reshape(self.num_coeffs,)),3),self.polytimer)

            x_snap_val = np.polyval(np.polyder(np.poly1d(coeffs_x.reshape(self.num_coeffs,)),4),self.polytimer)
            y_snap_val = np.polyval(np.polyder(np.poly1d(coeffs_y.reshape(self.num_coeffs,)),4),self.polytimer)
            z_snap_val = np.polyval(np.polyder(np.poly1d(coeffs_z.reshape(self.num_coeffs,)),4),self.polytimer)

            launchflag = self.started_flag and self.got_first_odom_flag

            yaw_x_vector = self.targets[self.active_segment,0]-self.odom_position[0,0]
            yaw_y_vector = self.targets[self.active_segment,1]-self.odom_position[1,0]


            if (np.absolute(yaw_x_vector) <= 0.75 and np.absolute(yaw_y_vector) <=0.75):
                if self.active_segment+1 < self.numtargets:
                    #yaw_x_vector = self.targets[self.active_segment+1,0]+self.target_orientation[self.active_segment+1,0]-self.odom_position[0,0]
                    yaw_x_vector = self.targets[self.active_segment+1,0]-self.odom_position[0,0]
                    #yaw_y_vector = self.targets[self.active_segment+1,1]+self.target_orientation[self.active_segment+1,1]-self.odom_position[1,0]
                    yaw_y_vector = self.targets[self.active_segment+1,1]-self.odom_position[1,0]
                    yaw_ref  = np.arctan2(yaw_y_vector,yaw_x_vector)
                else:
                    yaw_ref  = 0;
            else:
                yaw_ref  = np.arctan2(yaw_y_vector,yaw_x_vector)



            self.last_x_val = x_val
            self.last_y_val = y_val
            self.last_z_val = z_val
            self.last_x_vel_val = x_dot_val
            self.last_y_vel_val = y_dot_val
            self.last_z_vel_val = z_dot_val
            self.last_x_acc_val = x_dot_dot_val
            self.last_y_acc_val = y_dot_dot_val
            self.last_z_acc_val = z_dot_dot_val

            msg2 = target_dect_msg()
            msg2.header.stamp = rospy.Time.now()
            msg2.position_ref = Vector3(x_val,y_val,z_val)
            msg2.velocity_ref = Vector3(x_dot_val,y_dot_val,z_dot_val)
            msg2.accel_ref = Vector3(x_dot_dot_val,y_dot_dot_val,z_dot_dot_val)
            msg2.jerk_ref = Vector3(x_jerk_val,y_jerk_val,z_jerk_val)
            msg2.snap_ref = Vector3(x_snap_val,y_snap_val,z_snap_val)
            msg2.yaw_ref[0] = yaw_ref
            msg2.yaw_ref[1] = 0
            msg2.yaw_ref[2] = 0
            msg2.launch_flag = launchflag
            msg2.speed = np.linalg.norm(np.array([[x_dot_val],[y_dot_val],[z_dot_val]]))
            msg2.collide_state = self.collide_state
            self.traj_pub_complete.publish(msg2)

        pass

    def Traj_generator(self,ros_data):
        if self.new_segment != 0:

            start_time = time.time()
            if self.time_optim_flag != 0:
                pass

            elif self.active_segment+1 > self.numtargets:
                pass

            else:
                pose = self.target_orientation[self.active_segment:self.active_segment+3,0:3]
                specify_final_der = False
                final_der = np.zeros((int(deg),4))
                k_t = 50
                step_der = 0.01
                single_segment = True

                if not self.coefficients_x.any() and not self.coefficients_y.any() and not self.coefficients_z.any():
                    initial_der = np.zeros((3,deg))
                    px = np.zeros((self.num_coeffs,1)).reshape(self.num_coeffs,)
                    py = np.zeros((self.num_coeffs,1)).reshape(self.num_coeffs,)
                    pz = np.zeros((self.num_coeffs,1)).reshape(self.num_coeffs,)
                    polytimer = self.polytimer
                    segment_tracker = self.segment_tracker
                else:
                    segment_tracker = self.segment_tracker
                    polytimer = self.polytimer
                    if polytimer+1/self.rate_calc>self.traj_time_seg[segment_tracker] and (segment_tracker+1)<np.size(self.traj_time_seg,0):
                        segment_tracker = segment_tracker+1
                        if self.traj_time_seg[segment_tracker-1]-(polytimer+1/self.rate_calc)<self.traj_time_seg[segment_tracker]:
                            polytimer = self.traj_time_seg[segment_tracker]-(self.traj_time_seg[segment_tracker-1]-(polytimer+1/self.rate_calc))
                            segment_tracker = segment_tracker+1
                        else:
                            polytimer = self.traj_time_seg[segment_tracker-1]-(polytimer+1/self.rate_calc)
                    else:
                        polytimer = polytimer+1/self.rate_calc


                    px = np.poly1d(self.coefficients_x_calc.reshape(self.num_coeffs,))
                    py = np.poly1d(self.coefficients_y_calc.reshape(self.num_coeffs,))
                    pz = np.poly1d(self.coefficients_z_calc.reshape(self.num_coeffs,))
                    p1x = np.polyder(px,1)
                    p1x = np.polyval(p1x,polytimer)
                    p2x = np.polyder(px,2)
                    p2x = np.polyval(p2x,polytimer)
                    p3x = np.polyder(px,3)
                    p3x = np.polyval(p3x,polytimer)
                    p4x = np.polyder(px,4)
                    p4x = np.polyval(p4x,polytimer)
                    p1y = np.polyder(py,1)
                    p1y = np.polyval(p1y,polytimer)
                    p2y = np.polyder(py,2)
                    p2y = np.polyval(p2y,polytimer)
                    p3y = np.polyder(py,3)
                    p3y = np.polyval(p3y,polytimer)
                    p4y = np.polyder(py,4)
                    p4y = np.polyval(p4y,polytimer)
                    p1z = np.polyder(pz,1)
                    p1z = np.polyval(p1z,polytimer)
                    p2z = np.polyder(pz,2)
                    p2z = np.polyval(p2z,polytimer)
                    p3z = np.polyder(pz,3)
                    p3z = np.polyval(p3z,polytimer)
                    p4z = np.polyder(pz,4)
                    p4z = np.polyval(p4z,polytimer)
                    initial_der = np.array([[p1x,p2x,p3x,p4x],[p1y,p2y,p3y,p4y],[p1z,p2z,p3z,p4z]])

                if self.first_start:
                    self.first_start = False
                    self.last_x_val = self.initial_position[0,0]
                    self.last_y_val = self.initial_position[1,0]
                    self.last_z_val = self.initial_position[2,0]
                else:
                    last_x = self.last_x_val
                    last_y = self.last_y_val
                    last_z = self.last_z_val
                    self.last_x_val = np.polyval(px,polytimer)
                    self.last_y_val = np.polyval(py,polytimer)
                    self.last_z_val = np.polyval(pz,polytimer)


                if self.active_segment+1 > self.numtargets-1:
                    time_seg = np.array([np.divide(np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position),np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1)))*self.time_corr[2*self.active_segment]])
                    time_corr = np.array([self.time_corr[2*self.active_segment+1]])

                    waypoint = np.append(np.array([[self.last_x_val],[self.last_y_val],[self.last_z_val]]).reshape(1,3),self.targets[self.active_segment:self.active_segment+1,0:3],axis = 0)
                    pose = self.target_orientation[self.active_segment:self.active_segment+1,0:3]
                    single_segment = True
                    speed = self.avg_spd[self.active_segment:self.active_segment+1]

                elif self.active_segment+2 > self.numtargets-1:
                    time_seg = np.array([np.divide(np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position),np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1)))*self.time_corr[2*self.active_segment]])

                    if self.corr_const_flag == True:
                        time_corr = np.array([[self.time_corr[2*self.active_segment+1]],[self.time_corr[2*(self.active_segment+1)+1]]])
                        time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[2*self.active_segment]],[self.time_corr[2*(self.active_segment+1)]]])
                    else:
                        time_corr = np.array([[self.time_corr[self.active_segment+1]],[self.time_corr[(self.active_segment+1)+1]]])
                        time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[self.active_segment]],[self.time_corr[(self.active_segment+1)]]])

                    waypoint = np.append(np.array([[self.last_x_val],[self.last_y_val],[self.last_z_val]]).reshape(1,3),self.targets[self.active_segment:self.active_segment+2,0:3],axis = 0)
                    pose = self.target_orientation[self.active_segment:self.active_segment+2,0:3]
                    single_segment = True
                    speed = self.avg_spd[self.active_segment:self.active_segment+2]

                #elif self.active_segment+3 > self.numtargets-1:
                #    last_pos = np.array([self.last_x_val,self.last_y_val,self.last_z_val])
                    #time_seg = np.array([np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/self.avg_spd])
                #    time_seg = np.array([np.divide(np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-last_pos.reshape(3,1)),np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1)))*self.time_corr[2*self.active_segment]])

                #    if corr_const_flag == True:
                #        time_corr = np.array([[self.time_corr[2*self.active_segment+1]],[self.time_corr[2*(self.active_segment+1)+1]]])
                #        time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[2*self.active_segment]],[self.time_corr[2*(self.active_segment+1)]],[self.time_corr[2*(self.active_segment+2)]]])
                #    else:
                #        time_corr = np.array([[self.time_corr[self.active_segment+1]],[self.time_corr[(self.active_segment+1)+1]]])
                #        time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[self.active_segment]],[self.time_corr[(self.active_segment+1)]],[self.time_corr[2*(self.active_segment+2)]]])

                #    waypoint = np.append(np.array([[self.last_x_val],[self.last_y_val],[self.last_z_val]]).reshape(1,3),self.targets[self.active_segment:self.active_segment+3,0:3],axis = 0)
                #    pose = self.target_orientation[self.active_segment:self.active_segment+3,0:3]
                #    single_segment = True
                #    speed = self.avg_spd[self.active_segment:self.active_segment+3]
                else:
                    print("Case 4")
                    #time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/self.avg_spd],[np.linalg.norm(self.targets[self.active_segment+1,0:3].reshape(3,1)-self.targets[self.active_segment,0:3].reshape(3,1))/self.avg_spd]])
                    #time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[2*self.active_segment]],[self.time_corr[2*(self.active_segment+1)]]])
                    #waypoint = np.append(self.odom_position.reshape(1,3),self.targets[self.active_segment:self.active_segment+2,0:3],axis = 0)
                    #if corr_const_flag == True:
                        #time_corr = np.array([[self.time_corr[2*self.active_segment+1]],[self.time_corr[2*(self.active_segment+1)+1]]])
                        #time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[2*self.active_segment]],[self.time_corr[2*(self.active_segment+1)]],[self.time_corr[2*(self.active_segment+2)]],[self.time_corr[2*(self.active_segment+3)]]])
                    #else:
                        #time_corr = np.array([[self.time_corr[self.active_segment+1]],[self.time_corr[(self.active_segment+1)+1]]])
                        #time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[self.active_segment]],[self.time_corr[(self.active_segment+1)]]])

                    #waypoint = np.append(np.array([[self.last_x_val],[self.last_y_val],[self.last_z_val]]).reshape(1,3),self.targets[self.active_segment:self.active_segment+4,0:3],axis = 0)
                    #pose = self.target_orientation[self.active_segment:self.active_segment+4,0:3]
                    #speed = self.avg_spd[self.active_segment:self.active_segment+4]
                    #single_segment = True

                    time_seg = np.array([np.divide(np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position),np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1)))*self.time_corr[2*self.active_segment]])

                    if self.corr_const_flag == True:
                        time_corr = np.array([[self.time_corr[2*self.active_segment+1]],[self.time_corr[2*(self.active_segment+1)+1]]])
                        time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[2*self.active_segment]],[self.time_corr[2*(self.active_segment+1)]]])
                    else:
                        time_corr = np.array([[self.time_corr[self.active_segment+1]],[self.time_corr[(self.active_segment+1)+1]]])
                        time_seg = np.array([[np.linalg.norm(self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)/np.linalg.norm(self.targets_3d_ref_dist[self.active_segment,0:3].reshape(3,1))*self.time_corr[self.active_segment]],[self.time_corr[(self.active_segment+1)]]])

                    waypoint = np.append(np.array([[self.last_x_val],[self.last_y_val],[self.last_z_val]]).reshape(1,3),self.targets[self.active_segment:self.active_segment+2,0:3],axis = 0)
                    pose = self.target_orientation[self.active_segment:self.active_segment+2,0:3]
                    single_segment = True
                    speed = self.avg_spd[self.active_segment:self.active_segment+2]


                #self.traj_time_seg = time_seg

                traj = trajectory_generation.Trajectory_generation(self.deg,initial_der,final_der,waypoint,pose,time_seg,self.time_optim_flag,self.corr_const_flag,self.corridor_radius,self.num_points_corr,self.corr_length,time_corr,k_t,step_der,single_segment,specify_final_der,speed)
                if self.mode == 1:
                    cost,sol,sol2,sol3 = traj.solvemat()
                elif self.mode == 2:
                    cost,sol,sol2,sol3 = traj.solvemat_spd()

                now = rospy.get_time()

                print("time")
                print(now)

                print(traj.waypoint)
                print("Time segments")
                print(traj.time_segments)
                print("Active target for visual target detection")
                print(self.active_target)
                print("Active trajectory segment for planning")
                print(self.active_segment)

                delta = time.time()-start_time
                print(delta)
                test_start = time.time()
                if delta < 1/self.rate_calc:
                    time.sleep(1/self.rate_calc-delta)
                    print(time.time()-test_start)
                    if cost < 5000.0:
                        self.new_segment = False
                        self.new_segment_result = True
                        self.coefficients_x = sol
                        self.coefficients_y = sol2
                        self.coefficients_z = sol3
                        self.traj_time_seg = traj.time_segments
                    else:
                        self.new_segment = False
                        self.new_segment_result = False
                else:
                    self.new_segment = False
                    self.new_segment_result = False




    def getclosesttarget(self,estimate):
        closest_target = 100000
        idx = 0
        for i in range(0,self.numtargets):
            distance_to_target = np.linalg.norm(self.targets[i,0:3].reshape(3,1)-estimate)

            # todo: add code for time between target for recalcultation using distance to target and max_speed

            #print(idx)
            #print(self.targets[i,0:3].reshape(3,1))
            #print(estimate)
            #print(estimate)
            #print(distance_to_target)
            if distance_to_target < closest_target and distance_to_target < self.threshold_distance_target:
                self.active_target = i
                #closest_target = distance_to_target
                idx = idx +1
        if idx == 0:
            print("No targets found (Outlier Detected)")
            self.active_target = -1
        #print(idx)


    def Imagecallback(self,ros_data):
        #print("Got image")
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        actual_time = time.time()
        #print(actual_time-self.start_time)
        #print(actual_time)
        if not self.started_flag and (actual_time-self.start_time)>self.start_threshhold:
            self.started_flag = True
            self.first_start = True
            self.new_segment = True
            self.polytimer = 0
            print("First message sent")

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")  #color image with blue-green-red color order
        detector_object = detector.Detector(cv_image)
        detector_object.load_model()  # fyc ADD-ON
        detector_object.run()
        if self.showimage:
            cv2.imshow('cv_img', detector_object.img)
            cv2.waitKey(2)
        if self.active_segment < self.numtargets:
            # add corridor constraint using pose and corridor length
            distance_to_target = np.linalg.norm(np.dot(self.target_orientation[self.active_segment:self.active_segment+1,0:3],self.corr_length/2.).reshape(3,1)+self.targets[self.active_segment,0:3].reshape(3,1)-self.odom_position)
            #print(distance_to_target)
            if distance_to_target < self.corr_length/1:
                self.new_segment = True
                if (self.active_segment < self.num_coeffs):
                    self.active_segment = self.active_segment+1

                print("Next target distance only")
                #print(distance_to_target)
                #print(self.active_segment-1)
                #print(self.active_segment)
                time_now = rospy.get_time()
                print(time_now)

            if detector_object.found_rect_flag and self.got_first_odom:
                rot_mat = np.zeros((3,3))
                cv2.Rodrigues(detector_object.rvec,rot_mat)
                # Go from camera to world coordinate
                R_trans = np.array([[0, 0, 1],[-1, 0, 0],[0, -1, 0]])
                R_base = np.array([[0, -1, 0],[0, 0, 1],[-1, 0,0]])
                rot_mat_total =np.dot(np.dot(self.odom_rot[0:3,0:3],np.dot(R_trans,rot_mat)),R_base)
                # Get ROtation matrix of actual position with correct rotation matrixes(transform from object to cam (rodrigues), then transform form cam to world (R_trans), then multiply by odom)
                # Get x vector from rotation matrix and average
                T_cam_cg= np.array([[0.137],[0.0],[-0.04]])
                position_estimate = self.odom_position + np.dot(self.odom_rot[0:3,0:3],np.dot(R_trans,detector_object.tvec))+T_cam_cg
                self.getclosesttarget(position_estimate)
                #print(self.active_target)

                if self.targets[self.active_target,3] == 0 and self.active_target != -1:
                    self.ref_target[self.active_target:self.active_target+1,0:3] = position_estimate.reshape(1,3).copy()
                    self.targets[self.active_target,3] = 1
                    self.targets[self.active_target:self.active_target+1,0:3]=position_estimate.reshape(1,3)
                    # pose_sensor
                    self.target_orientation[self.active_target:self.active_target+1,0:3] = rot_mat_total[0:3,0].reshape(1,3).copy()
                    #self.new_segment = True
                    #print("New segment")

                if self.active_segment != self.active_target and distance_to_target < self.corr_length:
                    if self.active_segment != self.active_target:
                        self.active_segment = self.active_segment+1
                        print("New target")

                elif self.active_target != -1:
                    self.targets[self.active_target:self.active_target+1,0:3] = ((np.dot(self.targets[self.active_target:self.active_target+1,0:3].reshape(3,1),self.targets[self.active_target,4])+position_estimate)/(self.targets[self.active_target,4]+1)).reshape(1,3)
                    avg_rot = (np.dot(self.target_orientation[self.active_target:self.active_target+1,0:3],self.targets[self.active_target,4])+rot_mat_total[0:3,0].reshape(1,3).copy())/(self.targets[self.active_target,4]+1)
                    avg_rot = avg_rot/np.linalg.norm(avg_rot)
                    self.target_orientation[self.active_target,0:3] = avg_rot
                    self.targets[self.active_target,4] = self.targets[self.active_target,4]+1

                    distance_to_ref = np.linalg.norm(self.ref_target[self.active_target:self.active_target+1,0:3].reshape(3,1)-self.targets[self.active_target:self.active_target+1,0:3].reshape(3,1))
                    #print("distance_to_ref")
                    #print(distance_to_ref)
                    #print(self.targets[self.active_target:self.active_target+1,0:3])
                    if distance_to_ref > self.threshold_dist_ref_recalculate and distance_to_target > self.corr_length:
                        self.ref_target[self.active_target:self.active_target+1,0:3] = self.targets[self.active_target:self.active_target+1,0:3]
                        self.new_segment = True
                        print("Recalculate New segment")
                        #print(distance_to_ref)

                    if self.active_segment != self.active_target and distance_to_target < self.corr_length:
                        self.active_segment = self.active_segment+1
                        self.new_segment = True
                        print("New target")





                # Todo: Add trajectory recalculation routine with condition



                #print("Active target")
                #print(self.active_target)
                #print(self.targets[self.active_target:self.active_target+1,0:3])
                #print(self.target_orientation[self.active_target:self.active_target+1,0:3])
                #print("Estimate")
                #print(self.targets[0:3,0:3])
                #print(self.target_orientation)


    def OdomCallback(self,ros_data):
        #print("Got odometry")
        if not self.got_first_odom_flag:
            self.got_first_odom_flag = True
            print("Got First odom")
            #print(ros_data.pose.pose.position.y)

        self.odom_position[0,0] = ros_data.pose.pose.position.x
        self.odom_position[1,0] = ros_data.pose.pose.position.y
        self.odom_position[2,0] = ros_data.pose.pose.position.z
        self.odom_rot = quaternion_matrix([ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y, ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w])

        if not self.got_first_odom:
            self.got_first_odom = True
            self.initial_position[0,0] = ros_data.pose.pose.position.x
            self.initial_position[1,0] = ros_data.pose.pose.position.y
            self.initial_position[2,0] = ros_data.pose.pose.position.z
            self.initial_orientation = self.odom_rot[:,0:1]
            #print(self.initial_position[0,0])





if __name__ == '__main__':
    rospy.init_node('target_detector_node', anonymous=True)



    number_of_tracks = 6
    track_number = 1
    speed = 3
    showimage = False
    corr_length = 2.0
    #corr_length = 1.0
    corr_const_flag = True
    mode = 2
    # Parse user inputs
    #print("number of inputs")
    #print(len(sys.argv))
    #print(sys.argv)

    for i in range(0,len(sys.argv)):
        if sys.argv[i] == '-track':
            track_number = sys.argv[i+1]
        elif sys.argv[i] == "-image":
            if sys.argv[i+1] == "True":
                showimage = True
            elif sys.argv[i+1] == "False":
                showimage = False
        elif sys.argv[i] == "-mode":
            mode = float(sys.argv[i+1])
        elif sys.argv[i] == "-speed":
            print("Here")
            print(sys.argv[i+1])
            desired_speed = float(sys.argv[i+1])
            if desired_speed == 1.0:
                speed = 1.0
            elif desired_speed == 1.5:
                speed = 1.5
            elif desired_speed == 2.0:
                speed = 2.0
            elif desired_speed == 2.5:
                speed = 2.5
            elif desired_speed == 3.0:
                speed = 3.0
            elif desired_speed == 3.5:
                speed = 3.5
            elif desired_speed == 4.0:
                speed = 4.0
            elif desired_speed == 4.5:
                speed = 4.5
            elif desired_speed == 5.0:
                speed = 5.0
            elif desired_speed == 5.5:
                speed = 5.5
            elif desired_speed == 6.0:
                speed = 6.0
            elif desired_speed == 6.5:
                speed = 6.5
            elif desired_speed == 7.0:
                print("Speed set")
                speed = 7.0
            elif desired_speed == 8.0:
                print("Speed set")
                speed = 8.0

    if track_number == "race_track_multigp1":
        targets_3d =  np.array([[56,0,6],[28,14,6],[56,28,6],[-14,14,6],[0,0,6]])
        targets_3d_orientation = np.array([[1,0,0],[-1,0,0],[1,0,0],[0,-1,0],[1,0,0]])
        if speed == 3.0:

            time_corr_2 = np.array([23.7009,0,13.8524,0,13.0337,0,16.0115,0,3.4159,23.7009])
            avg_spd = np.array([0.53568,1.31,0.6658,3.3525,0.53568])
        elif speed == 3.5:
            time_corr_2 = np.array([20.3161,0,11.8722,0,11.1721,0,13.7234,0,2.9286,20.3161])
            avg_spd = np.array([0.62464,1.5284,0.77644,3.9103,0.62464])
        elif speed == 4.0:
            time_corr_2 = np.array([17.776,0,10.3893,0,9.7746,0,12.0091,0,2.5619,17.776])
            avg_spd = np.array([5.03296208482785,2.59328320188137,1.93789637485059,3.65972147633657,4.52210775551006])
        elif speed == 4.5:
            time_corr_2 = np.array([15.8005,0,9.2349,0,8.6889,0,10.6746,0,2.2774,15.8005])
            avg_spd = np.array([0.8035,1.965,0.99874,5.0285,0.8035])
        elif speed == 5.0:
            time_corr_1 = np.array([10.9134,0.6,6.261,0.6,6.261,0.6,14.5169,0.6,3.5765,10.9134])
            time_corr_2 = np.array([14.221,0,8.3105,0,7.8208,0,9.6064,0,2.05,14.221])
            avg_spd = np.array([0.89251,2.1834,1.1092,5.5863,0.89251])
        elif speed == 5.5:
            time_corr_2 = np.array([12.9271,0,7.5563,0,7.1097,0,8.7333,0,1.8633,12.9271])
            avg_spd = np.array([0.98257,2.4015,1.2204,6.1459,0.98257])
        elif speed == 6.0:
            time_corr_1 = np.array([9.0945,0.5,5.2175,0.5,5.2175,0.5,12.0974,0.5,2.9804,9.0945])
            time_corr_2 = np.array([11.8504,0,6.9261,0,6.5169,0,8.0052,0,1.7086,11.8504])
            avg_spd = np.array([1.0712,2.6201,1.331,6.7027,1.0712])
        elif speed == 6.5:
            time_corr_2 = np.array([10.939,0,6.3926,0,6.0166,0,7.3891,0,1.5771,10.939])
            avg_spd = np.array([8.18526961888769,4.21382166412823,3.15119069871887,5.94660185523431,7.34869179335693])
        elif speed == 7.0:
            time_corr_2 = np.array([7.20899438202028,0.533333333333333,4.17399355799961,0.533333333333333,4.17399355799961,0.533333333333333,9.73242004847715,0.533333333333333,2.29120055865915,0.533333333333333])
            avg_spd = np.array([8.85809344281019,5.20726476406774,4.16017097164568,6.53831290641927,8.24467425629215])

    elif track_number == "race_track_multigp2":
        targets_3d =  np.array([[28,0,3],[56,0,3],[28,-14,3],[28,-19,3],[0,0,3]])
        targets_3d_orientation = np.array([[1,0,0],[1,0,0],[1,0,0],[-1,0,0],[1,0,0]])
        if speed == 3.0:
            time_corr_2 = np.array([12.2328,0,7.8878,0,12.0586,0,3.4716,0,6.4401,12.2328])
            avg_spd = np.array([5.4883,0.20775,2.0066,0.61608,5.4883])
        elif speed == 3.5:
            time_corr_2 = np.array([10.4852,0,6.7611,0,10.3359,0,2.9757,0,5.5199,10.4852])
            avg_spd = np.array([6.403,0.24242,2.341,0.71867,6.403])
        elif speed == 4.0:
            time_corr_2 = np.array([9.1746,0,5.9157,0,9.0443,0,2.6036,0,4.8299,9.1746])
            avg_spd = np.array([7.3177,0.27686,2.6755,0.82148,7.3177])
        elif speed == 4.5:
            time_corr_2 = np.array([8.1552,0,5.2585,0,8.0392,0,2.3145,0,4.2932,8.1552])
            avg_spd = np.array([8.2325,0.31156,3.0098,0.92392,8.2325])
        elif speed == 5.0:
            time_corr_2 = np.array([7.3396,0,4.7328,0,7.2351,0,2.0833,0,3.8637,7.3396])
            avg_spd = np.array([9.1472,0.34634,3.3441,1.0261,9.1472])
        elif speed == 5.5:
            time_corr_2 = np.array([6.6724,0,4.3024,0,6.5777,0,1.8932,0,3.5129,6.6724])
            avg_spd = np.array([10.0619,0.3807,3.679,1.1303,10.0619])
        elif speed == 6.0:
            time_corr_2 = np.array([6.1165,0,3.9441,0,6.0292,0,1.7357,0,3.22,6.1165])
            avg_spd = np.array([10.9766,0.41564,4.0132,1.2323,10.9766])
        elif speed == 6.5:
            time_corr_2 = np.array([5.6459,0,3.6405,0,5.5657,0,1.6023,0,2.9722,5.6459])
            avg_spd = np.array([11.8913,0.44996,4.3476,1.3348,11.8913])
        elif speed == 7.0:
            time_corr_2 = np.array([5.2426,0,3.3804,0,5.1681,0,1.4879,0,2.7599,5.2426])
            avg_spd = np.array([12.8061,0.48459,4.682,1.4374,12.8061])

    elif track_number == "race_track_multigp3":
        targets_3d =  np.array([[28,14.0,3],[0,14,3],[-21,14,3],[-28,14,3],[0,0,3]])
        targets_3d_orientation = np.array([[0,-1,0],[0,1,0],[1,0,0],[-1,0,0],[1,0,0]])
        if speed == 3.0:
            time_corr_2 = np.array([19.2046,0,7.9862,0,3.6766,0,1.7208,0,6.9871,19.2046])
            avg_spd = np.array([0.57719,5.9331,4.905,3.1134,0.57719])
        elif speed == 3.5:
            time_corr_2 = np.array([16.4612,0,6.8452,0,3.1514,0,1.475,0,5.989,16.4612])
            avg_spd = np.array([0.6734,6.922,5.7225,3.6324,0.6734])
        elif speed == 4.0:
            time_corr_2 = np.array([14.4027,0,5.9903,0,2.7576,0,1.2907,0,5.2403,14.4027])
            avg_spd = np.array([0.76952,7.9106,6.5397,4.1507,0.76952])
        elif speed == 4.5:
            time_corr_2 = np.array([12.8017,0,5.3255,0,2.4517,0,1.1481,0,4.6566,12.8017])
            avg_spd = np.array([0.86588,8.8982,7.3543,4.6642,0.86588])
        elif speed == 5.0:
            time_corr_2 = np.array([11.5221,0,4.7923,0,2.2061,0,1.0326,0,4.1921,11.5221])
            avg_spd = np.array([0.9619,9.8883,8.1745,5.1882,0.9619])
        elif speed == 5.5:
            time_corr_2 = np.array([10.4761,0,4.3552,0,2.0051,0,0.93821,0,3.8119,10.4761])
            avg_spd = np.array([1.0582,10.8785,8.9949,5.7125,1.0582])
        elif speed == 6.0:
            time_corr_2 = np.array([9.6026,0,3.9928,0,1.8382,0,0.86028,0,3.4939,9.6026])
            avg_spd = np.array([1.1544,11.8666,9.8108,6.2286,1.1544])
        elif speed == 6.5:
            time_corr_2 = np.array([8.8633,0,3.6864,0,1.6971,0,0.79443,0,3.2244,8.8633])
            avg_spd = np.array([1.2506,12.8544,10.6259,6.7427,1.2506])
        elif speed == 7.0:
            time_corr_2 = np.array([8.2302,0,3.423,0,1.5757,0,0.7375,0,2.9945,8.2302])
            avg_spd = np.array([1.3466,13.8438,11.4448,7.2646,1.3466])

    elif track_number == "race_track_multigp4":
        targets_3d =  np.array([[-14,-14,3],[-28,-28,3],[14,14,3],[-42,0,3],[0,0,3]])
        targets_3d_orientation = np.array([[-1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0]])
        if speed == 3.0:
            time_corr_2 = np.array([15.2485,0,9.051,0,19.2884,0,16.5619,0,6.1507,15.2485])
            avg_spd = np.array([3.19,0.31872,1.8035,1.5189,3.19])
        elif speed == 3.5:
            time_corr_2 = np.array([13.0701,0,7.7565,0,16.536,0,14.1939,0,5.2724,13.0701])
            avg_spd = np.array([3.7216,0.3721,2.104,1.7716,3.7216])
        elif speed == 4.0:
            time_corr_2 = np.array([11.4364,0,6.7873,0,14.4686,0,12.4194,0,4.6138,11.4364])
            avg_spd = np.array([4.2533,0.42519,2.4045,2.0244,4.2533])
        elif speed == 4.5:
            time_corr_2 = np.array([10.1657,0,6.0329,0,12.8614,0,11.0394,0,4.101,10.1657])
            avg_spd = np.array([4.785,0.47841,2.7051,2.2775,4.785])
        elif speed == 5.0:
            time_corr_2 = np.array([9.1491,0,5.4296,0,11.5751,0,9.9355,0,3.6909,9.1491])
            avg_spd = np.array([5.3166,0.53154,3.0057,2.5306,5.3166])
        elif speed == 5.5:
            time_corr_2 = np.array([8.3173,0,4.9362,0,10.5227,0,9.0323,0,3.3553,8.3173])
            avg_spd = np.array([5.8483,0.58463,3.3062,2.7837,5.8483])
        elif speed == 6.0:
            time_corr_2 = np.array([7.6242,0,4.5247,0,9.646,0,8.2796,0,3.0758,7.6242])
            avg_spd = np.array([6.38,0.63785,3.6068,3.0367,6.38])
        elif speed == 6.5:
            time_corr_2 = np.array([7.0378,0,4.1765,0,8.9042,0,7.6426,0,2.8392,7.0378])
            avg_spd = np.array([6.9116,0.69109,3.9073,3.2896,6.9116])
        elif speed == 7.0:
            time_corr_2 = np.array([6.5351,0,3.8783,0,8.268,0,7.0967,0,2.6365,6.5351])
            avg_spd = np.array([7.4433,0.74417,4.2079,3.5426,7.4433])

    elif track_number == "race_track_multigp5":
        targets_3d =  np.array([[28,28,3],[10.5,45.5,3],[0,35,3],[7,28,3],[0,0,3]])
        targets_3d_orientation = np.array([[0,1,0],[-1,0,0],[0,-1,0],[1,0,0],[1,0,0]])
        if speed == 3.0:
            time_corr_2 = np.array([15.4191,0,7.6286,0,7.8487,0,2.968,0,5.4853,15.4191])
            avg_spd = np.array([3.974,3.9443,2.7991,3.6386,3.974])
        elif speed == 3.5:
            time_corr_2 = np.array([13.2166,0,6.5386,0,6.7278,0,2.5438,0,4.7014,13.2166])
            avg_spd = np.array([4.6362,4.6018,3.2659,4.2453,4.6362])
        elif speed == 4.0:
            time_corr_2 = np.array([11.5645,0,5.7215,0,5.8865,0,2.2259,0,4.1139,11.5645])
            avg_spd = np.array([5.2986,5.2591,3.7321,4.8516,5.2986])
        elif speed == 4.5:
            time_corr_2 = np.array([10.2797,0,5.0854,0,5.2329,0,1.9784,0,3.6566,10.2797])
            avg_spd = np.array([5.9608,5.9167,4.1993,5.4585,5.9608])
        elif speed == 5.0:
            time_corr_2 = np.array([9.2517,0,4.5769,0,4.7097,0,1.7806,0,3.2909,9.2517])
            avg_spd = np.array([6.6231,6.5741,4.6657,6.0648,6.6231])
        elif speed == 5.5:
            time_corr_2 = np.array([8.4105,0,4.161,0,4.2814,0,1.6188,0,2.9918,8.4105])
            avg_spd = np.array([7.2855,7.2315,5.1321,6.6712,7.2855])
        elif speed == 6.0:
            time_corr_1 = np.array([5.5209,0.42857,3.2325,0.42857,1.8183,0.42857,1.1112,0.42857,4.2474,5.5209])
            time_corr_2 = np.array([7.7097,0,3.8143,0,3.9244,0,1.4838,0,2.7427,7.7097])
            avg_spd = np.array([7.9478,7.889,5.5991,7.2782,7.9478])
        elif speed == 6.5:
            time_corr_2 = np.array([7.1167,0,3.5207,0,3.6228,0,1.3696,0,2.5316,7.1167])
            avg_spd = np.array([8.61,8.5465,6.0659,7.8846,8.61])
        elif speed == 7.0:
            time_corr_2 = np.array([6.6083,0,3.2693,0,3.3638,0,1.2718,0,2.3509,6.6083])
            avg_spd = np.array([9.2724,9.2038,6.5322,8.4911,9.2724])

    elif track_number == "circle":
    	targets_3d = np.array([[15,15,3],[310000000,0,3],[15,-15,3],[4.39,-10.61,3]])
    	targets_3d_orientation = np.array([[1,0,0],[0,-1,0],[-1,0,0],[-0.707,0.707,0]])
    	if speed == 4:
    		time_corr_1 = np.array([5.1738,0.5,4.9497,0.5,4.9497,0.5,2.4087,0.49992])
    		time_corr_3 = np.array([5.3461, 0,5.3033, 0,5.3033, 0,2.8706,0])
    		avg_spd_2 = np.array([4.0952,4.0103,4.0053,4.0024])
    		time_corr_4 = np.array([9.0774,0.40954,4.0365,0.35428,3.6242,0.30843,1.4125,0.259])
    		avg_spd = np.array([4.8876,5.6493,6.4898,7.7339])
    		time_corr_2 = np.array([8.8284, 0,4.3662, 0,3.937, 0,1.6917,0])
    	elif speed == 5:
    		time_corr_1 = np.array([4.139,0.4,3.9598,0.4,3.9598,0.4,1.9269,0.39994])
    		time_corr_3 = np.array([4.2769, 0,4.2426, 0,4.2426, 0,2.2965,0])
    		avg_spd_2 = np.array([5.119,5.0129,5.0066,5.003])
    		time_corr_4 = np.array([7.2618,0.32764,3.2293,0.28343,2.8994,0.24675,1.13,0.20721])
    		avg_spd = np.array([6.1095,7.0616,8.1121,9.6672])
    		time_corr_2 = np.array([7.0627, 0,3.4929, 0,3.1496, 0,1.3534,0])
    	elif speed == 6:
    		time_corr_1 = np.array([3.4492,0.33333,3.2998,0.33333,3.2998,0.33333,1.6058,0.33328])
    		time_corr_3 = np.array([3.5641, 0,3.5355, 0,3.5355, 0,1.9137,0])
    		avg_spd_2 = np.array([6.1428,6.0155,6.008,6.0036])
    		time_corr_4 = np.array([6.0515,0.27303,2.691,0.23619,2.4162,0.20563,0.94165,0.17267])
    		avg_spd = np.array([7.3314,8.4739,9.7346,11.6007])
    		time_corr_2 = np.array([5.8856, 0,2.9108, 0,2.6247, 0,1.1278,0])
    	elif speed == 7:
    		time_corr_1 = np.array([2.9565,0.28571,2.8284,0.28571,2.8284,0.28571,1.3764,0.28567])
    		time_corr_3 = np.array([3.0549, 0,3.0305, 0,3.0305, 0,1.6403,0])
    		avg_spd_2 = np.array([7.1666,7.0181,7.0093,7.0042])
    		time_corr_4 = np.array([5.1871,0.23402,2.3066,0.20245,2.071,0.17625,0.80713,0.148])
    		avg_spd = np.array([8.5533,9.8863,11.357,13.5342])
    		time_corr_2 = np.array([5.0448, 0,2.495, 0,2.2497, 0,0.9667,0])
    	elif speed == 8:
    		time_corr_1 = np.array([2.5869,0.25,2.4749,0.25,2.4749,0.25,1.2043,0.24996])
    		time_corr_3 = np.array([2.673, 0,2.6517, 0,2.6517, 0,1.4353,0])
    		avg_spd_2 = np.array([8.1904,8.0207,8.0106,8.0048])
    		time_corr_4 = np.array([4.5386,0.20477,2.0183,0.17714,1.8121,0.15422,0.70624,0.1295])
    		avg_spd = np.array([9.7752,11.2986,12.9793,15.4675])
    		time_corr_2 = np.array([4.4142, 0,2.1831, 0,1.9685, 0,0.84587,0])

    elif track_number == "infiniti":
    	targets_3d = np.array([[20,-12.31,3],[35,0,3],[20,12.31,3],[-20,-12.31,3],[-35,0,3],[-20,12.31,4.5],[0,0,6]])
    	targets_3d_orientation = np.array([[1,0,0],[0,1,0],[-1,0,0],[-1,0,0],[0,1,0],[1,0,0],[0.707,-0.707,0]])
    	if speed == 4:
    		time_corr_1 = np.array([5.0161,0.5,4.4994,0.5,4.4994,0.5,11.3196,0.5,4.4994,0.5,4.515,0.5,5.4286,0.49992])
    		time_corr_3 = np.array([5.2296, 0,4.8511, 0,4.8511, 0,11.7424, 0,4.8511, 0,4.8656, 0,5.8832,0])
    		avg_spd_2 = np.array([4.0717,4.0235,4.0038,4.0057,4.0154,4.0069,3.9989])
    		time_corr_4 = np.array([9.2425,0.42797,4.392,0.49352,4.2273,0.39065,8.3082,0.43504,4.6331,0.58078,5.121,0.5119,4.2474,0.26623])
    		avg_spd = np.array([4.6736,4.0651,5.1234,4.6033,3.4602,3.9151,7.5127])
    		time_corr_2 = np.array([9.1386, 0,4.7955, 0,4.6212, 0,8.5, 0,5.0666, 0,5.5748, 0,4.5774,0])
    	elif speed == 5:
    		time_corr_1 = np.array([4.0129,0.4,3.5995,0.4,3.5995,0.4,9.0557,0.4,3.5995,0.4,3.612,0.4,4.3429,0.39994])
    		time_corr_3 = np.array([4.1837, 0,3.8809, 0,3.8809, 0,9.3939, 0,3.8809, 0,3.8925, 0,4.7065,0])
    		avg_spd_2 = np.array([5.0897,5.0294,5.0048,5.0071,5.0193,5.0086,4.9987])
    		time_corr_4 = np.array([7.3939,0.34238,3.5136,0.39482,3.3818,0.31252,6.6466,0.34804,3.7065,0.46463,4.0969,0.40952,3.3979,0.21298])
    		avg_spd = np.array([5.842,5.0814,6.4043,5.754,4.3252,4.8938,9.391])
    		time_corr_2 = np.array([7.3109, 0,3.8364, 0,3.697, 0,6.8, 0,4.0533, 0,4.4598, 0,3.6619,0])
    	elif speed == 6:
    		time_corr_1 = np.array([3.3441,0.33333,2.9996,0.33333,2.9996,0.33333,7.5464,0.33333,2.9996,0.33333,3.01,0.33333,3.6191,0.33328])
    		time_corr_3 = np.array([3.4864, 0,3.2341, 0,3.2341, 0,7.8283, 0,3.2341, 0,3.2437, 0,3.9221,0])
    		avg_spd_2 = np.array([6.1076,6.0353,6.0057,6.0086,6.0232,6.0103,5.9984])
    		time_corr_4 = np.array([6.1616,0.28532,2.928,0.32902,2.8182,0.26044,5.5388,0.29003,3.0887,0.38719,3.4141,0.34128,2.8316,0.17748])
    		avg_spd = np.array([7.0104,6.0976,7.685,6.9049,5.1902,5.8725,11.2693])
    		time_corr_2 = np.array([6.0924, 0,3.197, 0,3.0808, 0,5.6667, 0,3.3777, 0,3.7165, 0,3.0516,0])
    	elif speed == 7:
    		time_corr_1 = np.array([2.8664,0.28571,2.5711,0.28571,2.5711,0.28571,6.4684,0.28571,2.5711,0.28571,2.58,0.28571,3.1021,0.28567])
    		time_corr_3 = np.array([2.9884, 0,2.7721, 0,2.7721, 0,6.7099, 0,2.7721, 0,2.7803, 0,3.3618,0])
    		avg_spd_2 = np.array([7.1256,7.0412,7.0067,7.01,7.027,7.012,6.9981])
    		time_corr_4 = np.array([5.2814,0.24456,2.5097,0.28201,2.4156,0.22323,4.7476,0.24859,2.6475,0.33188,2.9263,0.29252,2.427,0.15213])
    		avg_spd = np.array([8.1788,7.1139,8.9659,8.0557,6.0553,6.8513,13.1476])
    		time_corr_2 = np.array([5.2221, 0,2.7403, 0,2.6407, 0,4.8572, 0,2.8952, 0,3.1856, 0,2.6157,0])
    	elif speed == 8:
    		time_corr_1 = np.array([2.5081,0.25,2.2497,0.25,2.2497,0.25,5.6598,0.25,2.2497,0.25,2.2575,0.25,2.7143,0.24996])
    		time_corr_3 = np.array([2.6148, 0,2.4256, 0,4.4256, 0,5.8712, 0,2.4256, 0,2.4328, 0,2.9416,0])
    		avg_spd_2 = np.array([8.1435,8.0471,8.0076,8.0114,8.0309,8.0137,7.9979])
    		time_corr_4 = np.array([4.6212,0.21399,2.196,0.24676,2.1137,0.19533,4.1541,0.21752,2.3165,0.29039,2.5605,0.25595,2.1237,0.13311])
    		avg_spd = np.array([9.3472,8.1302,10.2467,9.2065,6.9203,7.8301,15.0257])
    		time_corr_2 = np.array([4.5693, 0,2.3978, 0,2.3106, 0,4.25, 0,2.5333, 0,2.7874, 0,2.2887,0])


    elif track_number == "race_track_alphapilot":
    	targets_3d = np.array([[26.4022,-0.8187,6.0065],[62.441,-1.2123,6.4625],[51.4731,-15.9103,2.2465],[30.7087,-16.2152,1.6937],[10.8633,-25.3087,2.9299],[-8.7464,-19.4695,1.8031],[10.2938,-10.7502,2.2465],[32.444,-30.6908,1.7199],[53.6232,-28.0486,2.5979]])
    	targets_3d_orientation = np.array([[1,0,0],[0.707,-0.707,0],[-1,0,0],[-1,0,0],[-1,0,0],[0,1,0],[1,0,0],[1,0,0],[1,0,0]])
    	if speed == 4:
    		time_corr_1 = np.array([6.5289,0.5,8.5841,0.49992,4.5241,0.5,4.6938,0.5,5.0167,0.5,4.8154,0.5,4.9068,0.5,7.0884,0.5,4.8451,0.5])
    		time_corr_3 = np.array([6.7723, 0,9.0109, 0,4.7044, 0,5.1935, 0,5.4661, 0,5.1229, 0,5.2366, 0,7.4521, 0,5.3404,0])
    		avg_spd_2 = np.array([4.0147,4.0627,4.0069,4.0026,4.0071,4.0034,3.9977,4.0071,4.0053])
    		time_corr_4 = np.array([8.3444,0.26105,5.6978,0.64663,6.5293,0.44554,4.3502,0.73516,7.095,0.37896,4.3268,0.94411,4.1251,0.34158,7.3367,0.8617,2.9116,0.17145])
    		avg_spd = np.array([7.6584,3.1389,4.4841,2.7254,5.2732,2.3655,5.8541,2.4319,11.6678])
    		time_corr_2 = np.array([8.3335, 0,6.0621, 0,6.8856, 0,4.9271, 0,7.4785, 0,4.8314, 0,4.599, 0,7.8194, 0,3.3625,0])
    	elif speed == 5:
    		time_corr_1 = np.array([5.2231,0.4,6.8672,0.39994,3.6193,0.4,3.755,0.4,4.0134,0.4,3.8523,0.4,3.9255,0.4,5.6707,0.4,3.8761,0.4])
    		time_corr_3 = np.array([5.4178, 0,7.2088, 0,6.7635, 0,4.1548, 0,4.3729, 0,4.0983, 0,4.1893, 0,5.9617, 0,4.2723,0])
    		avg_spd_2 = np.array([5.0184,5.0783,5.0086,5.0033,5.0089,5.0042,4.9971,5.0089,5.0066])
    		time_corr_4 = np.array([6.6416,0.20631,4.5038,0.52254,5.1994,0.35613,3.4734,0.60474,5.6587,0.29428,3.4038,0.71545,3.1656,0.24637,5.3902,1.8908,2.0147,0.11448])
    		avg_spd = np.array([9.6903,3.89,5.6101,3.3147,6.7903,3.0773,8.1157,2.9582,17.4739])
    		time_corr_2 = np.array([6.8027, 0,4.8799, 0,5.6802, 0,3.9579, 0,6.2874, 0,3.8338, 0,3.5238, 0,6.0882, 0,2.3855,0])
    	elif speed == 6:
    		time_corr_1 = np.array([4.3526,0.33333,5.7227,0.33328,3.016,0.33333,3.1292,0.33333,3.3445,0.33333,3.2103,0.33333,3.2712,0.33333,4.7256,0.33333,3.23,0.33333])
    		time_corr_3 = np.array([4.5149, 0,6.0073, 0,6.1363, 0,3.4623, 0,3.6441, 0,3.4153, 0,3.4911, 0,4.9681, 0,3.5602,0])
    		avg_spd_2 = np.array([6.0221,6.094,6.0103,6.0039,6.0107,6.0051,5.9966,6.0107,6.0079])
    		time_corr_4 = np.array([5.5829,0.17501,3.8194,0.43126,4.4152,0.29462,2.7644,0.42038,4.8109,0.26542,2.9319,0.63869,2.7735,0.23459,4.9035,0.48729,1.9342,0.11898])
    		avg_spd = np.array([11.4235,4.7046,6.7818,4.7577,7.5288,3.5127,8.5245,4.2092,16.8142])
    		time_corr_2 = np.array([5.5696, 0,4.0621, 0,4.631, 0,3.1206, 0,5.0205, 0,3.2777, 0,3.1006, 0,5.2059, 0,2.2115,0])
    	elif speed == 7:
    		time_corr_1 = np.array([3.7308,0.28571,4.9052,0.28567,2.5852,0.28571,2.6821,0.28571,2.8667,0.28571,2.7517,0.28571,2.8039,0.28571,4.0505,0.28571,2.7686,0.28571])
    		time_corr_3 = np.array([3.8699, 0,5.1491, 0,4.6882, 0,2.9677, 0,3.1235, 0,2.9274, 0,2.9923, 0,4.2583, 0,3.0516,0])
    		avg_spd_2 = np.array([7.0257,7.1097,7.0121,7.0046,7.0125,7.0059,6.996,7.0124,7.0093])
    		time_corr_4 = np.array([4.7579,0.14842,3.2426,0.37087,3.7407,0.25534,2.4464,0.38336,4.0412,0.22991,2.5448,0.56132,2.385,0.19923,4.196,0.45178,1.6613,0.099798])
    		avg_spd = np.array([13.4702,5.4752,7.8245,5.221,8.6922,4.0216,10.0373,4.5799,20.0454])
    		time_corr_2 = np.array([4.7499, 0,3.4536, 0,3.944, 0,2.7552, 0,4.2648, 0,2.8366, 0,2.661, 0,4.4549, 0,1.9083,0])
    	elif speed == 8:
    		time_corr_1 = np.array([3.2644,0.25,4.292,0.24996,2.262,0.25,2.3469,0.25,2.5084,0.25,2.4077,0.25,2.4534,0.25,3.5442,0.25,2.4225,0.25])
    		time_corr_3 = np.array([3.3862, 0,4.5055, 0,4.3522, 0,2.5967, 0,2.7331, 0,2.5615, 0,2.6183, 0,3.726, 0,2.6702,0])
    		avg_spd_2 = np.array([8.0294,8.1254,8.0138,8.0053,8.0143,8.0067,7.9954,8.0142,8.0106])
    		time_corr_4 = np.array([4.1845,0.1304,2.8432,0.32312,3.2821,0.2234,2.1494,0.34902,3.5492,0.19369,2.1865,0.48232,2.0834,0.17517,3.6715,0.38051,1.4558,0.088416])
    		avg_spd = np.array([15.332,6.2815,8.9435,5.7366,10.3169,4.6592,11.4159,5.4121,22.626])
    		time_corr_2 = np.array([4.1661, 0,3.0284, 0,3.4499, 0,2.4326, 0,3.7386, 0,2.4393, 0,2.3244, 0,3.9034, 0,1.6669,0])

    targets_3d_ref_dist = np.zeros(targets_3d.shape)
    targets_3d_ref_dist[0,0:3] = targets_3d[0,0:3]-np.array([1.5,0.0,0.3])
    for i in range(1,targets_3d.shape[0]):
        targets_3d_ref_dist[i,0:3] = targets_3d[i,0:3]-targets_3d[i-1,0:3]


    threshold_distance_target = 3.0

    #avg_spd = np.array([8.65404597785762,5.66407027554245,6.16496381236746,5.82934808952812,5.67681420891770,3.84245028138229,4.13602312036667,4.28553010358012,6.19433063092415,7.56758703714550,12.5946452646451])
    #avg_spd = speed
    deg = 4

    corridor_radius = 0.05
    num_points_corr = 8.0

    if mode == 1:
        time_corr = time_corr_1
    elif mode == 2:
        time_corr = time_corr_3

    start_threshhold = 8
    print(speed)
    print(time_corr)
    print(targets_3d)
    tg = track_trajectory_gen(targets_3d,targets_3d_orientation,showimage,threshold_distance_target,avg_spd,deg,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,targets_3d_ref_dist,start_threshhold,mode)
    #print(time_corr.shape)
    rospy.spin()
