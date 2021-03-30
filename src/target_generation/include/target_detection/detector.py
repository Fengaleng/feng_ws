#!/usr/bin/env python

import numpy as np
import cv2
import tensorflow as tf


class Detector:

    def __init__(self,img):
        self.img = img

    # def SortCorners(self,corners):
    #     i =1
    #
    #     while i < corners.shape[0]:
    #       j = i
    #       while j > 0 and corners[j-1][1]>corners[j][1]:
    #         #copy_corners = corners[j]
    #         tmp = corners[j].copy()
    #         corners[j]=corners[j-1].copy()
    #         corners[j-1] = tmp.copy()
    #         j = j-1
    #       i = i+1
    #     # swap two first
    #     if corners[0][0] > corners[1][0]:
    #         tmp = corners[0].copy()
    #         corners[0]=corners[1].copy()
    #         corners[1] = tmp.copy()
    #     # swap two last
    #     if corners[3][0] > corners[2][0]:
    #         tmp = corners[2].copy()
    #         corners[2]=corners[3].copy()
    #         corners[3] = tmp.copy()
    #
    #
    # def GetContours(self):
    #     self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
    #     self.gray = cv2.GaussianBlur(self.gray,(5,5),1)
    #     re,thresh = cv2.threshold(self.img,127,255,2)
    #     self.edges = cv2.Canny(self.gray,100,200)
    #     _,self.contours, self.hierarchy = cv2.findContours(self.edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #
    # def PolyArea(self,polygon_array):
    #     area = 0.
    #     for point in range(0,polygon_array.shape[0]-1):
    #       area = area + polygon_array[point][0][0]*polygon_array[point+1][0][1]-polygon_array[point][0][1]*polygon_array[point+1][0][0]
    #
    #     area = area + polygon_array[polygon_array.shape[0]-1][0][0]*polygon_array[0][0][1]-polygon_array[polygon_array.shape[0]-1][0][1]*polygon_array[0][0][0]
    #     area = abs(area)/2.
    #     return area
    #
    # def GetPose(self):
    #     self.found_rect_flag = False
    #     self.corners = []
    #     approx_tmp = 0
    #     approx2_tmp = 0
    #     rvec_tmp = 1000000*np.ones((3,1))
    #     tvec_tmp = 1000000*np.ones((3,1))
    #     self.tvec =rvec_tmp.copy()
    #     self.rvec = tvec_tmp.copy()
    #     for cnt in range(len(self.contours)):
    #       self.approx = cv2.approxPolyDP(self.contours[cnt],cv2.arcLength(self.contours[cnt], True)*0.02,True)
    #       if len(self.approx)==4 and self.hierarchy[0][cnt][2]!=-1 and self.hierarchy[0][cnt+1][2]!=-1:   # closed quadrangle
    #         if cnt+3< len(self.contours):
    #           self.approx2 = cv2.approxPolyDP(self.contours[cnt+3],cv2.arcLength(self.contours[cnt+3], True)*0.02,True)
    #           if len(self.approx2)==4 and self.hierarchy[0][cnt+3][3] == cnt+2: #quadrangle in another quadrangle
    #           # area condition
    #             area_outer = self.PolyArea(self.approx)
    #             area_inner = self.PolyArea(self.approx2)
    #             if area_inner == 0:
    #                 break;
    #             ratio = area_outer/area_inner
    #             if  ratio< 1.7 and ratio > 1.45:
    #
    #               # findcorners accurately
    #               stop_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30, 0.001)
    #               cv2.cornerSubPix(self.gray, self.approx.astype("f"), (5, 5), (-1, -1), stop_criteria)
    #               cv2.cornerSubPix(self.gray, self.approx2.astype("f"), (5, 5), (-1, -1), stop_criteria)
    #               # Draw found target on image
    #
    #               self.SortCorners(self.approx.reshape(4,2))
    #               self.SortCorners(self.approx2.reshape(4,2))
    #               self.corners = np.append(self.approx.reshape(4,2),self.approx2.reshape(4,2),axis = 0)
    #               #print(self.corners)
    #               K = np.float64([[381.36246688113556, 0, 320.5],
    #                             [0, 381.36246688113556, 160.5],
    #                             [0.0,0.0,      1.0]])
    #               dist_coef = np.zeros(5)
    #               size_outer_area = 2.25
    #               size_inner_area = 1.8281
    #               depth = 0.
    #               target_3d = np.float32([[-size_outer_area/2., size_outer_area/2., depth],[size_outer_area/2., size_outer_area/2., depth],[size_outer_area/2., -size_outer_area/2., depth],   [-size_outer_area/2., -size_outer_area/2., depth],[-size_inner_area/2.,size_inner_area/2.,depth],[size_inner_area/2.,size_inner_area/2.,depth],[size_inner_area/2.,-size_inner_area/2.,depth],[-size_inner_area/2.,-size_inner_area/2.,depth]])
    #               flag_type = cv2.SOLVEPNP_ITERATIVE
    #               #print("Found")
    #               rvec = 0
    #               tvec =0
    #               _ret,rvec_tmp,tvec_tmp  = cv2.solvePnP(target_3d, self.corners.astype(np.float), K, dist_coef,flags=flag_type)
    #
    #
    #               if np.linalg.norm(tvec_tmp)<np.linalg.norm(self.tvec):
    #                   self.tvec = tvec_tmp.copy()
    #                   self.rvec = rvec_tmp.copy()
    #                   approx_tmp = self.approx.copy()
    #                   approx2_tmp = self.approx2.copy()
    #                   self.found_rect_flag = True
    #             else:
    #
    #               rvec = 0
    #               tvec = 0
    #
    #     if self.found_rect_flag:
    #         cv2.drawContours(self.img, [approx_tmp], -1, (0,255,0), 2)
    #         cv2.drawContours(self.img, [approx2_tmp], -1, (0,255,0), 2)
    #         #print(self.tvec)

    def load_model(self):
        abs_path = '/home/fechec/feng_ws/src/CNN'
        self.model = tf.keras.models.load_model(abs_path)
        self.model.summary()

    def GetPose(self):
        if self.img.shape is not (1, 200, 300, 3):
            self.img = self.img[np.newaxis, :, :]

        mean, _ = self.model(self.img)  # discard the covariance output
        tvec_tmp_spherical = mean[0, :3].numpy()
        psi = mean[0, -1].numpy()
        # go from spherical to cartesian
        rho = tvec_tmp_spherical[0]
        theta = tvec_tmp_spherical[1]
        phi = tvec_tmp_spherical[2]
        x = rho*np.sin(phi)*np.cos(theta)
        y = rho*np.sin(phi)*np.sin(theta)
        z = rho*np.cos(phi)

        self.tvec = np.array([x, y, z])  # tvec is an array
        self.rvec = np.array([np.cos(psi), np.sin(psi), 0])  # rvec is a value


    def run(self):
        #self.GetContours()
        self.GetPose()
