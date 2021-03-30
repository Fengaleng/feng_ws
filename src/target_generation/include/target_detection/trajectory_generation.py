#!/usr/bin/env python

import numpy as np
from qpsolvers import solve_qp
from scipy.linalg import block_diag
from scipy.sparse.linalg import cg,spsolve
from scipy import sparse
from qpsolvers import available_solvers
import scipy as sc
import time
import sys
np.set_printoptions(threshold=sys.maxsize)

class Trajectory_generation:
    def __init__(self,deg,inital_der,final_der,waypoint,pose,time_segments,time_optim_flag,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,k_t,step_der,single_segment,specify_final_der,speed):
        self.deg = deg
        self.inital_der = inital_der
        self.final_der = final_der
        self.waypoint = waypoint
        self.pose = pose
        self.time_segments = time_segments
        self.time_optim_flag = time_optim_flag
        self.corr_const_flag = corr_const_flag
        self.corridor_radius = corridor_radius
        self.num_points_corr = num_points_corr
        self.corr_length = corr_length
        self.time_corr = time_corr
        self.poly_deg = deg*2+1
        self.num_coeff = self.poly_deg+1
        self.num_segments = np.size(self.waypoint,0)-1
        self.k_t = k_t
        self.step_der = step_der
        size = int(self.num_coeff*self.num_segments)
        self.Q_x = np.zeros((size,size))
        self.A_eq_mat = np.zeros((size,size))
        size = int(self.deg+(self.deg+2)*self.num_segments)
        size2 = int(self.num_coeff*self.num_segments)
        self.A_eq = np.zeros((size,size2))
        self.b_eq_x = np.zeros((int(self.deg+1+2*self.num_segments),1))
        self.b_eq_y = np.zeros((self.deg+1+2*self.num_segments,1))
        self.b_eq_z = np.zeros((self.deg+1+2*self.num_segments,1))
        self.single_segment = single_segment
        self.specify_final_der = specify_final_der
        self.speed = speed


    def create_waypoint(self):
        if self.corr_const_flag != 0:
            if self.single_segment != 0:
                waypoint = np.zeros(((np.size(self.waypoint,0)-1)*2+2,3))
                waypoint[0,0:3] = self.waypoint[0,0:3]
                time_segments = np.zeros(np.size(waypoint,0)-1)
                time_segments[0]=self.time_segments[0]
                size = self.waypoint.shape[0]-1
                size_time = np.size(self.time_segments)-1
                #time_segments[time_segments.size-1]=self.time_segments[self.time_segments.size-1].copy()
                if self.time_corr.shape[0] > 1:
                    time_segments[time_segments.size-1]=self.time_corr[self.time_corr.size-1]
                else:
                    time_segments[time_segments.size-1]=self.time_corr[0]

                self.waypoint = np.delete(self.waypoint,0,axis = 0)

            else:
                waypoint = np.zeros((2+np.size(self.waypoint,0)*2,3))
                time_segments = np.zeros(np.size(waypoint,0)-1)
                size = self.waypoint.shape[0]
                time_segments[0]=self.time_segments[0]
                time_segments[time_segments.size-1]=self.time_segments[self.time_segments.size-1].copy()
                time_segments[time_segments.size-2]=self.time_corr
                size_time = np.size(self.time_segments)
            for i in range(0,size):
                direction = self.pose[i,0:3]/np.linalg.norm(self.pose[i,0:3])
                waypoint[1+2*i,0:3] = self.waypoint[i,0:3]-self.corr_length/2*np.transpose(direction)
                waypoint[2+2*i,0:3] = self.waypoint[i,0:3]+self.corr_length/2*np.transpose(direction)
            for j in range(0,size_time):
                if self.time_corr.shape[0] > 1:
                    time_segments[1+2*j] = self.time_corr[j]
                else:
                    time_segments[1+2*j] = self.time_corr[0]

                time_segments[2+2*j] = self.time_segments[j+1]
            self.waypoint = waypoint.copy()
            self.waypoint = np.delete(self.waypoint,self.waypoint.shape[0]-1,axis = 0)
            self.time_segments = time_segments.copy()
            self.time_segments = np.delete(self.time_segments,self.time_segments.shape[0]-2,axis = 0)
            self.corr_const_flag = False
        else:

            if self.single_segment != 0:
                pass
            else:

                self.waypoint = np.vstack((np.append([np.zeros(2)],[1]),self.waypoint,np.append([np.zeros(2)],[1])))
                self.time_segments = np.concatenate((self.time_segments[0:1,0:1],self.time_segments,self.time_segments[-1:,0:1]),axis=0)

        self.num_segments = np.size(self.waypoint,0)-1
        size = int(self.num_coeff*self.num_segments)
        self.Q_x = np.zeros((size,size))
        self.A_eq_mat = np.zeros((size,size))
        if self.single_segment != 0:
            size2 = int((self.deg+2)*self.num_segments)
            if self.specify_final_der != 0:

                size2 = int(self.deg+(self.deg+2)*self.num_segments)
                self.b_eq_x = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
                self.b_eq_y = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
                self.b_eq_z = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
            else:
                self.b_eq_x = np.zeros((int(self.deg+np.size(self.waypoint,0)),1))
                self.b_eq_y = np.zeros((self.deg+np.size(self.waypoint,0),1))
                self.b_eq_z = np.zeros((self.deg+np.size(self.waypoint,0),1))
                #self.b_eq_x = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
                #self.b_eq_y = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
                #self.b_eq_z = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
        else:
            size2 = int(self.deg+(self.deg+2)*self.num_segments)
            self.b_eq_x = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
            self.b_eq_y = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
            self.b_eq_z = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
        self.A_eq = np.zeros((size2,size))


    def set_params(self,deg,inital_der,final_der,waypoint,pose,time_segments,time_optim_flag,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,k_t,step_der):
        self.deg = deg
        self.inital_der = inital_der
        self.final_der = final_der
        self.waypoint = waypoint
        self.pose = pose
        self.time_segments = time_segments
        self.time_optim_flag = time_optim_flag
        self.corr_const_flag = corr_const_flag
        self.corridor_radius = corridor_radius
        self.num_points_corr = num_points_corr
        self.corr_length = corr_length
        self.time_corr = time_corr
        self.poly_deg = deg*2+1
        self.num_coeff = self.poly_deg+1
        self.num_segments = np.size(self.waypoint,0)-1
        self.k_t = k_t
        self.step_der = step_der

    def set_time(self,time_segments):
        self.time_segments = time_segments

    def Q_matrix(self):
        for i in range(0,self.num_segments):
            self.Q_x[self.num_coeff*i:self.num_coeff*(i+1),self.num_coeff*i:self.num_coeff*(i+1)] = self.costQ(i)


    def costQ(self,time_seg_idx):
        size = int(self.poly_deg+1)
        Q = np.zeros((size,size))
        for i in range(0,self.poly_deg+1):
            for j in range(0,self.poly_deg+1):
                if i >= self.deg and j >= self.deg:
                    S = 1
                    for m in range(0,self.deg):
                        S = S*(i-m)*(j-m)
                    Q[i,j] = 2*S*self.time_segments[time_seg_idx]**(i+j-2*self.deg+1)/(i+j-2*self.deg+1)
                else:
                    Q[i,j] = 0;
        return Q

    def A_constraint(self):
        print("Time")
        print(self.time_segments)
        vf = self.fAT(0)


        elm1 = np.append(self.fAO(), np.zeros((int(self.deg+1),int((self.num_segments-1)*self.num_coeff))),axis = 1)

        elm2 = np.append(vf[0,:], np.zeros((1,int((self.num_segments-1)*self.num_coeff))))

        self.A_eq[int(0):int(self.deg+1+1),int(0):int(self.num_coeff*self.num_segments)] = np.vstack((elm1,[elm2]))

        for i in range(2,self.num_segments+1):
            vf = self.fAT(i-1);
            elm1 =  np.concatenate((np.zeros((int(self.deg+1),int((i-2)*(self.poly_deg+1)))), self.fAT(i-2), -self.fAO(), np.zeros((int(self.deg+1),int((self.num_segments-i)*(self.poly_deg+1))))),axis = 1)
            if i == self.num_segments:
                elm2 = np.concatenate((np.zeros((1,int((i-1)*(self.poly_deg+1)))), [vf[0,:]]),axis = 1)
            else:
                elm2 = np.concatenate((np.zeros((1,int((i-1)*(self.poly_deg+1)))), [vf[0,:]], np.zeros((1,int((self.num_segments-i)*(self.poly_deg+1))))),axis = 1)

            self.A_eq[int((self.deg+2)*(i-1)):int((self.deg+2)*(i)),int(0):int(self.num_coeff*self.num_segments)] = np.append(elm1,elm2,axis=0)

# np.zeros((int(self.deg),int((self.num_segments-i)*(self.poly_deg+1))))
        if self.single_segment != 0:
            if self.specify_final_der != 0:
                if self.num_segments == 2:
                    self.A_eq[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),int(0):int(self.num_coeff*self.num_segments)] = np.concatenate((np.zeros((int(self.deg),int((self.poly_deg+1)))), vf[1:,:]),axis = 1)
                else:
                    self.A_eq[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),int(0):int(self.num_coeff*self.num_segments)] = np.concatenate((np.zeros((int(self.deg),int((i-1)*(self.poly_deg+1)))), vf[1:,:]),axis = 1)
        else:
            vO = self.fAO()
            if self.specify_final_der != 0:
                if self.num_segments == 2:
                    self.A_eq[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),int(0):int(self.num_coeff*self.num_segments)] = np.concatenate((np.zeros((int(self.deg),int((self.poly_deg+1)))), vf[1:,:]),axis = 1)
                else:
                    self.A_eq[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),int(0):int(self.num_coeff*self.num_segments)] = np.concatenate((np.zeros((int(self.deg),int((i-1)*(self.poly_deg+1)))), vf[1:,:]),axis = 1)
            else:
                if self.num_segments == 2:
                    self.A_eq[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),int(0):int(self.num_coeff*self.num_segments)] = np.concatenate((-vO[1:,:], vf[1:,:]),axis = 1)
                else:
                    self.A_eq[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),int(0):int(self.num_coeff*self.num_segments)] = np.concatenate((-vO[1:,:], np.zeros((int(self.deg),int((i-2)*(self.poly_deg+1)))), vf[1:,:]),axis = 1)





        # b_constraints



        #self.b_eq_x[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,0]], np.zeros((int(self.deg),1)), [self.waypoint[1,0]]),axis=None)

        self.b_eq_x[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,0]], self.inital_der[0:1,0:self.deg], [self.waypoint[1,0]]),axis=None)
        print(self.b_eq_x.shape)
        for i in range(2,self.num_segments+1):
            self.b_eq_x[int((self.deg+2)*(i-1)):int((self.deg+2)*(i)),0] = np.concatenate((np.zeros((self.deg+1,1)),[self.waypoint[i,0]]),axis=None)


        if self.single_segment != 0:
            if self.specify_final_der != 0:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,0:1]
        else:
            if self.specify_final_der != 0:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,0:1]
            else:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))


        self.b_eq_y[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,1]], self.inital_der[1:2,0:self.deg], [self.waypoint[1,1]]),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_y[int((self.deg+2)*(i-1)):int((self.deg+2)*(i)),0] = np.concatenate((np.zeros((self.deg+1,1)),self.waypoint[i,1]),axis=None)

        if self.single_segment != 0:
            pass
            if self.specify_final_der != 0:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,1:2]
        else:
            if self.specify_final_der != 0:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,1:2]
            else:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))

        #print(int(self.deg+2))
        ##print(self.waypoint[0,2])
        #print(self.waypoint[1,2])
        #print( self.inital_der[2:3,0:self.deg])
        #print(np.concatenate(([self.waypoint[0,2]], self.inital_der[2:3,0:self.deg], [self.waypoint[1,2]]),axis=None).shape)
        self.b_eq_z[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,2]], self.inital_der[2:3,0:self.deg], [self.waypoint[1,2]]),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_z[int((self.deg+2)*(i-1)):int((self.deg+2)*(i)),0] = np.concatenate((np.zeros((self.deg+1,1)),self.waypoint[i,2]),axis=None)

        if self.single_segment != 0:
            pass
            if self.specify_final_der != 0:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,2:3]
        else:
            if self.specify_final_der != 0:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,2:3]
            else:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))












    def A_corridor_constraint(self):

        A = np.zeros((int(6*np.floor(self.num_segments/2)*self.num_points_corr),int(self.num_segments*self.num_coeff*3)))
        b = np.zeros((int(6*np.floor(self.num_segments/2)*self.num_points_corr),1))
        idx = -5

        for i in range(0,self.num_segments-1,2):
            #print(i)
            r_T = np.zeros((3,self.num_coeff*3*self.num_segments))
            t = self.waypoint[i+2,0:3]-self.waypoint[i+1,0:3]
            t = t/np.linalg.norm(t)
            int_corridor_mat = np.array([[(1-t[0]**2),(-t[0]*t[1]),(-t[0]*t[2])],[(-t[0]*t[2]),(1-t[1]**2),(-t[1]*t[2])],[(-t[0]*t[2]),(-t[2]*t[1]),(1-t[2]**2)]])
            for j in range(0,int(self.num_points_corr)):

                idx = idx+6
                time = (j+1)/(1+self.num_points_corr)*self.time_segments[i+1]
                for ii in range(0,self.num_coeff):
                    r_T[0,int((i+1)*self.num_coeff+ii)] = time**(ii)
                    r_T[1,int(self.num_coeff*self.num_segments+(i+1)*self.num_coeff+ii)] = r_T[0,int((i+1)*self.num_coeff+ii)]
                    r_T[2,int(2*self.num_coeff*self.num_segments+(i+1)*self.num_coeff+ii)] = r_T[0,int((i+1)*self.num_coeff+ii)]

                A[int(idx-1):int(idx+2),:] = np.dot(int_corridor_mat,r_T)

                b[int(idx-1):int(idx+2),0:1] = np.dot(self.corridor_radius,np.ones((3,1)))+np.dot(int_corridor_mat,self.waypoint[i+1,0:3].reshape(3,1))

                A[int(idx+2):int(idx+5),:]=np.dot(-int_corridor_mat,r_T)

                b[int(idx+2):int(idx+5),0:1] = np.dot(self.corridor_radius,np.ones((3,1)))-np.dot(int_corridor_mat,self.waypoint[i+1,0:3].reshape(3,1))


        A_x = A[::3,0:self.num_coeff*self.num_segments]
        A_y = A[1::3,self.num_coeff*self.num_segments:2*self.num_coeff*self.num_segments]
        A_z = A[2::3,2*self.num_coeff*self.num_segments:3*self.num_coeff*self.num_segments]

        b_x = b[::3,0:1]
        b_y = b[1::3,0:1]
        b_z = b[2::3,0:1]

        empty_idx_x = np.where(~A_x.any(axis=1))[0]
        empty_idx_y = np.where(~A_y.any(axis=1))[0]
        empty_idx_z = np.where(~A_z.any(axis=1))[0]

        A_x_corr = np.delete(A_x,empty_idx_x,axis = 0)
        A_y_corr = np.delete(A_y,empty_idx_y,axis = 0)
        A_z_corr = np.delete(A_z,empty_idx_z,axis = 0)


        b_x_corr = np.delete(b_x,empty_idx_x,axis = 0)
        b_y_corr = np.delete(b_y,empty_idx_y,axis = 0)
        b_z_corr = np.delete(b_z,empty_idx_z,axis = 0)
        empty_idx = np.where(~A.any(axis=1))[0]
        #empty_idx_2 = np.where((A ==0).all(axis=1))
        A_corr = np.delete(A,empty_idx,axis = 0)
        b_corr = np.delete(b,empty_idx,axis = 0)


        self.A_corr = np.zeros(A_corr.shape)
        self.b_corr = np.zeros(b_corr.shape)



        if A_x_corr.size == 0:
            self.A_corr[0:A_y_corr.shape[0],self.num_segments*self.num_coeff:self.num_segments*self.num_coeff*2] = A_y_corr
            self.A_corr[A_y_corr.shape[0]:A_y_corr.shape[0]+A_z_corr.shape[0],self.num_segments*self.num_coeff*2:self.num_segments*self.num_coeff*3] = A_z_corr
            self.b_corr = np.vstack((b_y_corr,b_z_corr)).copy()
        elif A_y_corr.size == 0:
            self.A_corr[0:A_x_corr.shape[0],0:self.num_segments*self.num_coeff] = A_x_corr
            self.A_corr[A_x_corr.shape[0]:A_x_corr.shape[0]+A_z_corr.shape[0],self.num_segments*self.num_coeff*2:self.num_segments*self.num_coeff*3] = A_z_corr
            self.b_corr = np.vstack((b_x_corr,b_z_corr)).copy()
        elif A_z_corr.size == 0:
            self.A_corr[0:A_x_corr.shape[0],0:self.num_segments*self.num_coeff] = A_x_corr
            self.A_corr[A_x_corr.shape[0]:A_x_corr.shape[0]+A_y_corr.shape[0],self.num_segments*self.num_coeff:self.num_segments*self.num_coeff*2] = A_y_corr
            self.b_corr = np.vstack((b_x_corr,b_y_corr)).copy()
        else:
            self.A_corr = block_diag(A_x_corr,A_y_corr,A_z_corr).copy()
            self.b_corr = np.vstack((b_x_corr,b_y_corr,b_z_corr)).copy()







    def fAT(self,time_idx):
        AO = np.zeros((self.deg+1,self.poly_deg+1))
        for i in range(0,self.deg+1):
            for j in range(0,self.poly_deg+1):
                if j>= i:
                    S=1
                    for m in range(0,i):
                        S = S*(j-m)
                    AO[i,j]=S*self.time_segments[time_idx]**(j-i)
                else:
                    AO[i,j]=0.0

        return AO

    def fAO(self):
        AO = np.zeros((self.deg+1,self.poly_deg+1))
        for i in range(0,self.deg+1):
            for j in range(0,self.poly_deg+1):
                if i == j:
                    S=1
                    for m in range(0,i):
                        S = S*(i-m)
                    AO[i,j] = S
                else:
                    AO[i,j] = 0.0
        return AO


    def solve(self):
        start_time = time.time()
        self.create_waypoint()
        self.Q_matrix()
        self.A_constraint()

        P = block_diag(self.Q_x,self.Q_x,self.Q_x)
        q = np.zeros((self.num_segments*self.num_coeff*3))
        A = block_diag(self.A_eq,self.A_eq,self.A_eq)
        b = np.vstack((self.b_eq_x,self.b_eq_y,self.b_eq_z)).flatten()
        P = P+1*np.identity(P.shape[0])
        #print(np.linalg.cond(P))
        #print(np.linalg.cond(self.Q_x))
        #print(np.linalg.cond(self.Q_x+0.1*np.identity(self.Q_x.shape[0])))

        #print(P.shape)
        #print(q.shape)
        #print(A.shape)
        #print(b.shape)
        #print(P)
        #print(A)

        if self.time_optim_flag!=0:
            pass
        else:
            if self.corr_const_flag != 0:
                self.A_corridor_constraint()
                G = self.A_corr
                h = self.b_corr.flatten()
                #print(G.shape)
                #print(h.shape)
                sol = solve_qp(P,q,G,h,A,b,solver = "quadprog")
                print("modified cost")
                print(np.dot(np.dot(sol.transpose(),P),sol))
                print("cost")
                print(np.dot(np.dot(sol.transpose(),P-0.01*np.identity(P.shape[0])),sol))
                return sol
            else:
                #print("No corridor")
                P = self.Q_x+1*np.identity(self.Q_x.shape[0])
                q = np.zeros((self.num_segments*self.num_coeff))
                A = self.A_eq
                b = self.b_eq_x.flatten()
                print(np.linalg.cond(P))
                #print(P.shape)
                #print(q.shape)
                #print(A.shape)
                #print(b.shape)
                cases = [{'P': P, 'q': q, 'A': A, 'b':b}]
                for (i,case) in enumerate(cases):
                    solver = "quadprog"
                    sol = solve_qp(solver=solver, **case)
                    print("modified cost")
                    print(np.dot(np.dot(sol.transpose(),P),sol))
                    #print("cost")
                    #print(np.dot(np.dot(sol.transpose(),P-0.01*np.identity(P.shape[0])),sol))

                b = self.b_eq_y.flatten()
                cases2 = [{'P': P, 'q': q, 'A': A, 'b':b}]
                for (i,case2) in enumerate(cases2):
                    solver = "quadprog"
                    sol2 = solve_qp(solver=solver, **case2)
                    print("modified cost")
                    print(np.dot(np.dot(sol.transpose(),P),sol2))
                    #print("cost")
                    #print(np.dot(np.dot(sol.transpose(),P-0.01*np.identity(P.shape[0])),sol))

                b = self.b_eq_z.flatten()
                cases3 = [{'P': P, 'q': q, 'A': A, 'b':b}]
                for (i,case3) in enumerate(cases3):
                    solver = "quadprog"
                    sol3 = solve_qp(solver=solver, **case3)
                    print("modified cost")
                    print(np.dot(np.dot(sol.transpose(),P),sol3))
                    #print("cost")
                    #print(np.dot(np.dot(sol.transpose(),P-0.01*np.identity(P.shape[0])),sol))

                print("exec time:")
                print(time.time()-start_time)

                return sol,sol2,sol3

    def solver(self):
        P = block_diag(self.Q_x,self.Q_x,self.Q_x)
        q = np.zeros((self.num_segments*self.num_coeff*3))
        A = block_diag(self.A_eq,self.A_eq,self.A_eq)
        b = np.vstack((self.b_eq_x,self.b_eq_y,self.b_eq_z)).flatten()
        P = P+0.1*np.identity(P.shape[0])
        if self.corr_const_flag != 0:
            self.A_corridor_constraint()
            G = self.A_corr
            h = self.b_corr.flatten()
            sol = solve_qp(P,q,G,h,A,b,solver = "quadprog")
            return sol
        else:
            #print("No corridor")
            cases = [{'P': P, 'q': q, 'A': A, 'b':b}]
            for (i,case) in enumerate(cases):
                solver = "quadprog"
                sol = solve_qp(solver=solver, **case)
                return sol


    def A_constraint_mat(self):
        for i in range(0,self.num_segments):
            v0 = self.fAO()
            vf = self.fAT(i)
            self.A_eq_mat[self.num_coeff*i:self.num_coeff*(i+1),self.num_coeff*i:self.num_coeff*(i+1)] = np.vstack((v0,vf))



        self.b_eq_x[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,0]], self.inital_der[0:1,0:self.deg], [self.waypoint[1,0]]),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_x[self.deg+i,0] = self.waypoint[i,0]


        if self.single_segment != 0:
            if self.specify_final_der != 0:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,0:1]
        else:
            if self.specify_final_der != 0:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,0:1]
            else:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))


        self.b_eq_y[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,1]], self.inital_der[1:2,0:self.deg], [self.waypoint[1,1]]),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_y[self.deg+i,0] = self.waypoint[i,1]

        if self.single_segment != 0:
            pass
            if self.specify_final_der != 0:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,1:2]
        else:
            if self.specify_final_der != 0:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,1:2]
            else:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))

        #print(int(self.deg+2))
        ##print(self.waypoint[0,2])
        #print(self.waypoint[1,2])
        #print( self.inital_der[2:3,0:self.deg])
        #print(np.concatenate(([self.waypoint[0,2]], self.inital_der[2:3,0:self.deg], [self.waypoint[1,2]]),axis=None).shape)
        self.b_eq_z[0:int(self.deg+2),0] = np.concatenate(([self.waypoint[0,2]], self.inital_der[2:3,0:self.deg], [self.waypoint[1,2]]),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_z[self.deg+i,0] = self.waypoint[i,2]

        if self.single_segment != 0:
            pass
            if self.specify_final_der != 0:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,2:3]
        else:
            if self.specify_final_der != 0:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,2:3]
            else:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))


    def solvemat(self):
        start_time = time.time()
        self.create_waypoint()
        self.Q_matrix()
        self.A_constraint_mat()
        #print("exec time create mat:")
        #print(time.time()-start_time)
        #start_time = time.time()

        M = np.zeros((self.num_coeff*self.num_segments,self.num_coeff*self.num_segments-(self.deg+1)*(self.num_segments-1)))
        DF = self.b_eq_x
        length_DF = self.b_eq_x.shape[0]
        # Selector matrix
        for i in range(0,self.num_segments):
            if i == 0:
                M[0:self.deg+2,0:self.deg+2] = np.identity(self.deg+2)
                for j in range(self.deg+2,self.num_coeff):
                    M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-2]=1;
            else:
                for j in range(0,self.num_coeff):
                    if j == 0:
                        M[i*self.num_coeff+j,self.deg+1+i-1] = 1
                    elif j < self.deg+1:
                        M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-1] = 1
                    elif j == self.deg+1:
                        M[i*self.num_coeff+j,self.deg+i-1+2] = 1
                    else:
                        M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-2] = 1

        #P = block_diag(self.Q_x,self.Q_x,self.Q_x)
        P = self.Q_x
        P = P+0.01*np.identity(P.shape[0])
        print(self.A_eq_mat.shape)
        print(np.linalg.matrix_rank(self.A_eq_mat))
        # Dark magic


        length_M = (self.deg+1)*self.num_segments*2
        #print("exec time selector:")
        #print(time.time()-start_time)


        #R_int = sc.linalg.lstsq(A_full.T,P,overwrite_a = True,overwrite_b = True)
        #start_time = time.time()
        #print(np.linalg.cond(self.A_eq_mat))
        self.A_eq_mat = self.A_eq_mat+0.0015*np.identity(self.A_eq_mat.shape[0])
        #print(np.linalg.cond(self.A_eq_mat))
        R_int = sc.linalg.lstsq(self.A_eq_mat.T,P,lapack_driver = "gelsy")[0]
        #print("exec time 1:")
        #print(time.time()-start_time)
        #start_time = time.time()
        R = sc.linalg.lstsq(self.A_eq_mat.T,R_int.transpose(),lapack_driver = "gelsy")[0].transpose()
        #print("exec time 2:")
        #print(time.time()-start_time)
        #start_time = time.time()
        R = np.dot(np.dot(M.transpose(),R),M)
        #print("exec time 3:")
        #print(time.time()-start_time)
        #start_time = time.time()

        RPP = R[length_DF:length_M,length_DF:length_M]
        #print("exec time 4:")
        #print(time.time()-start_time)
        #start_time = time.time()

        RFP = R[0:length_DF,length_DF:length_M]
        #print("exec time 5:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T,lapack_driver = "gelsy")[0],DF)
        #print("exec time 6:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p1 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))),lapack_driver = "gelsy")[0]
        #print("exec time 7:")
        #print(time.time()-start_time)
        #start_time = time.time()


        DF = self.b_eq_y
        #print("exec time 8:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T,lapack_driver = "gelsy")[0],DF)
        #print("exec time 9:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p2 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))),lapack_driver = "gelsy")[0]
        #print("exec time 10:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DF = self.b_eq_z
        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T,lapack_driver = "gelsy")[0],DF)
        #print("exec time 11:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p3 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))),lapack_driver = "gelsy")[0]
        #print("exec time 12:")
        #print(time.time()-start_time)
        #start_time = time.time()

        print("exec time invert:")
        print(time.time()-start_time)
        print("cost")
        cost = np.dot(np.dot(p1.transpose(),self.Q_x),p1)+np.dot(np.dot(p2.transpose(),self.Q_x),p2)+np.dot(np.dot(p3.transpose(),self.Q_x),p3)
        print(cost)

        return cost,p1,p2,p3

    def A_constraint_mat_spd(self):
        for i in range(0,self.num_segments):
            v0 = self.fAO()
            vf = self.fAT(i)
            self.A_eq_mat[self.num_coeff*i:self.num_coeff*(i+1),self.num_coeff*i:self.num_coeff*(i+1)] = np.vstack((v0,vf))

        #self.b_eq_x =


        self.b_eq_x[0:int(self.deg+3),0] = np.concatenate(([self.waypoint[0,0]], self.inital_der[0:1,0:self.deg], [self.waypoint[1,0]],np.dot(self.speed[0],self.pose[0,0])),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_x[self.deg+2*i-1,0] = self.waypoint[i,0]
            self.b_eq_x[self.deg+2*i,0] = np.dot(self.speed[i-1],self.pose[i-1,0])

        if self.single_segment != 0:
            if self.specify_final_der != 0:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,0:1]
        else:
            if self.specify_final_der != 0:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,0:1]
            else:
                self.b_eq_x[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))


        self.b_eq_y[0:int(self.deg+3),0] = np.concatenate(([self.waypoint[0,1]], self.inital_der[1:2,0:self.deg], [self.waypoint[1,1]],np.dot(self.speed[0],self.pose[0,1])),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_y[self.deg+2*i-1,0] = self.waypoint[i,1]
            self.b_eq_y[self.deg+2*i,0] = np.dot(self.speed[i-1],self.pose[i-1,1])


        if self.single_segment != 0:
            pass
            if self.specify_final_der != 0:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,1:2]
        else:
            if self.specify_final_der != 0:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,1:2]
            else:
                self.b_eq_y[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))

        #print(int(self.deg+2))
        ##print(self.waypoint[0,2])
        #print(self.waypoint[1,2])
        #print( self.inital_der[2:3,0:self.deg])
        #print(np.concatenate(([self.waypoint[0,2]], self.inital_der[2:3,0:self.deg], [self.waypoint[1,2]]),axis=None).shape)
        self.b_eq_z[0:int(self.deg+3),0] = np.concatenate(([self.waypoint[0,2]], self.inital_der[2:3,0:self.deg], [self.waypoint[1,2]],np.dot(self.speed[0],0)),axis=None)

        for i in range(2,self.num_segments+1):
            self.b_eq_z[self.deg+2*i-1,0] = self.waypoint[i,2]
            self.b_eq_z[self.deg+2*i,0] = np.dot(self.speed[i-1],0)

        if self.single_segment != 0:
            pass
            if self.specify_final_der != 0:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,2:3]
        else:
            if self.specify_final_der != 0:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = self.final_der[:,2:3]
            else:
                self.b_eq_z[int((self.deg+2)*self.num_segments):int((self.deg+2)*self.num_segments+self.deg),0:1] = np.zeros((self.deg,1))


    def solvemat_spd(self):
        start_time = time.time()
        #self.create_waypoint()

        self.Q_matrix()
        self.A_constraint_mat_spd()

        ############New Method ###########################
        #P = self.Q_x

        #size = int(self.num_coeff*self.num_segments)
        #A_eq_mat_inv = np.zeros((size,size))
        #R_int = np.zeros((size,size))
        #for i in range(0,self.num_segments):
        #    A_diag = self.A_eq_mat[self.num_coeff*i:int(self.num_coeff*(i+0.5)),self.num_coeff*i:int(self.num_coeff*(i+0.5))]
        #    C = self.A_eq_mat[int(self.num_coeff*(i+0.5)):int(self.num_coeff*(i+1)),self.num_coeff*i:int(self.num_coeff*(i+0.5))]
        #    D = self.A_eq_mat[int(self.num_coeff*(i+0.5)):int(self.num_coeff*(i+1)),int(self.num_coeff*(i+0.5)):int(self.num_coeff*(i+1))]
        #    A_inv = np.divide(np.identity(int(self.num_coeff/2)),A_diag.diagonal())*np.identity(int(self.num_coeff/2))
        #    D_inv = np.linalg.inv(D)
        #    A_eq_mat_inv[self.num_coeff*i:int(self.num_coeff*(i+0.5)),self.num_coeff*i:int(self.num_coeff*(i+0.5))] = A_inv
        #    A_eq_mat_inv[int(self.num_coeff*(i+0.5)):self.num_coeff*(i+1),int(self.num_coeff*(i)):int(self.num_coeff*(i+0.5))] = -D_inv*C*A_inv
        #    A_eq_mat_inv[int(self.num_coeff*(i+0.5)):self.num_coeff*(i+1),int(self.num_coeff*(i+0.5)):self.num_coeff*(i+1)] = D_inv
        #    A_int = A_eq_mat_inv[self.num_coeff*i:int(self.num_coeff*(i+1)),self.num_coeff*i:int(self.num_coeff*(i+1))]
        #    P = self.Q_x[self.num_coeff*i:int(self.num_coeff*(i+1)),self.num_coeff*i:int(self.num_coeff*(i+1))]
        #    P = P+0.1*np.identity(P.shape[0])
        #    R_int2 = np.matmul(A_int.transpose(), P)
        #    R_int3 = np.matmul(R_int2,A_int)
        #    R_int[self.num_coeff*i:int(self.num_coeff*(i+1)),self.num_coeff*i:int(self.num_coeff*(i+1))] = R_int3
            #print(np.linalg.norm(A_eq_mat_inv[self.num_coeff*i:int(self.num_coeff*(i+1)),self.num_coeff*i:int(self.num_coeff*(i+1))]-np.linalg.inv(self.A_eq_mat[self.num_coeff*i:int(self.num_coeff*(i+1)),self.num_coeff*i:int(self.num_coeff*(i+1))])))
            #print(A_diag)
            #print(A_inv)
        ############New Method ###########################
        #R_int_meth2 = np.matmul(A_eq_mat_inv.transpose(),self.Q_x + 0.1*np.identity(self.Q_x.shape[0]))
        #R_int_meth2 = np.matmul(R_int_meth2,A_eq_mat_inv)
        #print("test")
        ##print(np.linalg.cond(A_eq_mat_inv))
        #print(np.linalg.cond(np.linalg.inv(self.A_eq_mat)))
        #print("exec time create mat:")
        #print(time.time()-start_time)
        #start_time = time.time()
        #print(self.time_segments)

        M = np.zeros((self.num_coeff*self.num_segments,self.num_coeff*self.num_segments-(self.deg+1)*(self.num_segments-1)))
        DF = self.b_eq_x
        length_DF = self.b_eq_x.shape[0]
        # Selector matrix
        for i in range(0,self.num_segments):
            if i == 0:
                M[0:self.deg+3,0:self.deg+3] = np.identity(self.deg+3)
                for j in range(self.deg+3,self.num_coeff):
                    M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-3]=1;
            else:
                for j in range(0,self.num_coeff):
                    if j == 0:
                        M[i*self.num_coeff+j,self.deg+1+2*i-1-1] = 1
                    elif j == 1:
                        M[i*self.num_coeff+j,self.deg+1+2*i-1-1+1] = 1
                    elif j < self.deg+1:
                        M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-1-1] = 1
                    elif j == self.deg+1:
                        M[i*self.num_coeff+j,self.deg+2*i-1-1+3] = 1
                    elif j == self.deg+2:
                        M[i*self.num_coeff+j,self.deg+2*i-1-1+4] = 1
                    else:
                        M[i*self.num_coeff+j,length_DF+(self.deg-1)*(i-1)+j-4] = 1

        #P = block_diag(self.Q_x,self.Q_x,self.Q_x)
        P = self.Q_x
        P = P+0.01*np.identity(P.shape[0])
        # Dark magic


        length_M = (self.deg+1)*self.num_segments*2
        #print("exec time selector:")
        #print(time.time()-start_time)








        #print(M)
        #R_int = sc.linalg.lstsq(A_full.T,P,overwrite_a = True,overwrite_b = True)
        #start_time = time.time()
        #print(np.linalg.cond(self.A_eq_mat))
        self.A_eq_mat = self.A_eq_mat+0.0015*np.identity(self.A_eq_mat.shape[0])
        #print(np.linalg.cond(self.A_eq_mat))
        #print("Rint")

        #print(np.linalg.cond(R_int))
        R_int = sc.linalg.lstsq(self.A_eq_mat.T,P,lapack_driver = "gelsy")[0]
        #R_int = sc.linalg.lstsq(self.A_eq_mat.T,P)[0]
        #R_int2 = sparse.linalg.lsqr(sparse.csc_matrix(self.A_eq_mat.T),sparse.csc_matrix(P))
        #print("exec time 1:")
        #print(time.time()-start_time)
        #start_time = time.time()
        R = sc.linalg.lstsq(self.A_eq_mat.T,R_int.transpose(),lapack_driver = "gelsy")[0].transpose()
        #R2 = sparse.linalg.spsolve(sparse.csc_matrix(self.A_eq_mat.T),sparse.csc_matrix(R)).todense()
        #print("exec time 2:")
        #print(time.time()-start_time)
        #start_time = time.time()
        #print(np.linalg.cond(R))
        R = np.matmul(np.matmul(M.transpose(),R),M)
        #print(np.linalg.cond(R))
        #R = R + 0.01*np.identity(R.shape[0])
        #R_2 = np.matmul(M.transpose(),R_int)
        #R_3 = np.matmul(R_2,M)
        #print(np.linalg.cond(R_3))
        #print(np.linalg.cond(R))

        #print("exec time 3:")
        #print(time.time()-start_time)
        #start_time = time.time()

        RPP = R[length_DF:length_M,length_DF:length_M]
        #RPP = RPP
        #RPP2 = R_3[length_DF:length_M,length_DF:length_M]
        #print(np.linalg.cond(RPP2))
        #print(np.linalg.cond(RPP))
        #print("exec time 4:")
        #print(time.time()-start_time)
        #start_time = time.time()

        RFP = R[0:length_DF,length_DF:length_M]
        #RFP2 = R_3[0:length_DF,length_DF:length_M]
        #print(np.linalg.cond(RFP2))
        #print(np.linalg.cond(RFP))
        #print("exec time 5:")
        #print(time.time()-start_time)
        #start_time = time.time()
        #q,r = sc.linalg.qr(RPP)
        #DP2 = -sparse.linalg.spsolve(sparse.csr_matrix(RPP),sparse.csr_matrix(np.matmul(RFP.T,DF)))
        #print(DP2)
        DP = -np.matmul(sc.linalg.lstsq(RPP,RFP.T,lapack_driver = "gelsy")[0],DF)
        #print(DP)

        #DP_test = sc.linalg.solve(RPP,-np.matmul(RFP.T,DF))
        #DP_test2 = sc.linalg.solve(r,np.matmul(q.T,np.matmul(-RFP.T,DF)))
        #print(np.linalg.cond(DP2))
        #print(np.linalg.cond(DP))
        #print(np.linalg.norm(DP_test-DP))
        #print(np.linalg.norm(DP_test2-DP))
        #print("exec time 6:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p1 = sc.linalg.lstsq(self.A_eq_mat,(np.matmul(M,np.vstack((DF,DP)))),lapack_driver = "gelsy")[0]

        #p12 = sparse.linalg.cgs(self.A_eq_mat,(np.matmul(M,np.vstack((DF,DP)))))[0]
        #print("exec time 7:")
        #print(time.time()-start_time)
        #print(np.linalg.norm(p12-p1))
        #start_time = time.time()


        DF = self.b_eq_y
        #print("exec time 8:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T,lapack_driver = "gelsy")[0],DF)
        #print("exec time 9:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p2 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))),lapack_driver = "gelsy")[0]
        #p22 = sparse.linalg.cgs(self.A_eq_mat,(np.matmul(M,np.vstack((DF,DP)))))[0]
        #print("exec time 7:")
        #print(time.time()-start_time)
        #print(np.linalg.norm(p22-p2))
        #print("exec time 10:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DF = self.b_eq_z
        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T,lapack_driver = "gelsy")[0],DF)
        #print("exec time 11:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p3= sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))),lapack_driver = "gelsy")[0]
        #p32 = sparse.linalg.cgs(self.A_eq_mat,(np.matmul(M,np.vstack((DF,DP)))))[0]
        #print("exec time 7:")
        #print(time.time()-start_time)
        #print(np.linalg.norm(p32-p3))
        #print("exec time 12:")
        #print(time.time()-start_time)
        #start_time = time.time()
        P = self.Q_x
        print("exec time invert:")
        print(time.time()-start_time)
        print("cost")
        cost = np.dot(np.dot(p1.transpose(),self.Q_x),p1)+np.dot(np.dot(p2.transpose(),self.Q_x),p2)+np.dot(np.dot(p3.transpose(),self.Q_x),p3)
        print(cost)
        #cost2 =  np.dot(np.dot(p1.transpose(),self.Q_x),p12)+np.dot(np.dot(p2.transpose(),self.Q_x),p22)+np.dot(np.dot(p3.transpose(),self.Q_x),p32)
        #print(cost2)
        #print("speed")
        #print(self.speed)
        return cost,p1,p2,p3


    def solvemat_time(self):
        start_time = time.time()
        #self.create_waypoint()
        self.Q_matrix()
        self.A_constraint_mat()
        #print("exec time create mat:")
        #print(time.time()-start_time)
        #start_time = time.time()
        #print(self.time_segments)

        M = np.zeros((self.num_coeff*self.num_segments,self.num_coeff*self.num_segments-(self.deg+1)*(self.num_segments-1)))
        DF = self.b_eq_x
        length_DF = self.b_eq_x.shape[0]
        # Selector matrix
        for i in range(0,self.num_segments):
            if i == 0:
                M[0:self.deg+2,0:self.deg+2] = np.identity(self.deg+2)
                for j in range(self.deg+2,self.num_coeff):
                    M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-2]=1;
            else:
                for j in range(0,self.num_coeff):
                    if j == 0:
                        M[i*self.num_coeff+j,self.deg+1+i-1] = 1
                    elif j < self.deg+1:
                        M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-1] = 1
                    elif j == self.deg+1:
                        M[i*self.num_coeff+j,self.deg+i-1+2] = 1
                    else:
                        M[i*self.num_coeff+j,length_DF+self.deg*(i-1)+j-2] = 1

        #P = block_diag(self.Q_x,self.Q_x,self.Q_x)
        P = self.Q_x

        A = block_diag(self.A_eq,self.A_eq,self.A_eq)
        b = np.vstack((self.b_eq_x,self.b_eq_y,self.b_eq_z))
        #P = P+0.01*np.identity(P.shape[0])
        # Dark magic


        length_M = (self.deg+1)*self.num_segments*2
        #print("exec time selector:")
        #print(time.time()-start_time)


        #R_int = sc.linalg.lstsq(A_full.T,P,overwrite_a = True,overwrite_b = True)
        #start_time = time.time()
        R_int = sc.linalg.lstsq(self.A_eq_mat.T,P)[0]
        #print("exec time 1:")
        #print(time.time()-start_time)
        #start_time = time.time()
        R = sc.linalg.lstsq(self.A_eq_mat.T,R_int.transpose())[0].transpose()
        #print("exec time 2:")
        #print(time.time()-start_time)
        #start_time = time.time()
        R = np.dot(np.dot(M.transpose(),R),M)
        #print("exec time 3:")
        #print(time.time()-start_time)
        #start_time = time.time()

        RPP = R[length_DF:length_M,length_DF:length_M]
        #print("exec time 4:")
        #print(time.time()-start_time)
        #start_time = time.time()

        RFP = R[0:length_DF,length_DF:length_M]
        #print("exec time 5:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T)[0],DF)
        #print("exec time 6:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p1 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))))[0]
        #print("exec time 7:")
        #print(time.time()-start_time)
        #start_time = time.time()


        DF = self.b_eq_y
        #print("exec time 8:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T)[0],DF)
        #print("exec time 9:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p2 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))))[0]
        #print("exec time 10:")
        #print(time.time()-start_time)
        #start_time = time.time()

        DF = self.b_eq_z
        DP = -np.dot(sc.linalg.lstsq(RPP,RFP.T)[0],DF)
        #print("exec time 11:")
        #print(time.time()-start_time)
        #start_time = time.time()

        p3 = sc.linalg.lstsq(self.A_eq_mat,(np.dot(M,np.vstack((DF,DP)))))[0]
        #print("exec time 12:")
        #print(time.time()-start_time)
        #start_time = time.time()

        #print("exec time invert:")
        #print(time.time()-start_time)
        #print("cost")
        #print(np.dot(np.dot(p1.transpose(),P),p1)+np.dot(np.dot(p2.transpose(),P),p2)+np.dot(np.dot(p3.transpose(),P),p3))

        return p1,p2,p3


if __name__ == '__main__':
    start_time = time.time()
    deg = 4
    inital_der = np.zeros((deg,4))
    inital_der[0,0] = 10
    final_der = np.zeros((deg,4))
    specify_final_der = False # can't have corridor and final_der
    #waypoint = np.array([[0,0,0],[20, 20, 0],[0, 40, 0],[-20,20,0 ],[-20, -40, 0],[0,0,0]])
    waypoint = np.array([[0,0,0],[20, 20, 0]])

    pose = np.array([[0,1,0],[-1,0,0],[0,-1,0],[1,0,0],[1,0,0]])
    #time_segments = np.array([[10],[10],[10],[10],[10]])
    #waypoint = np.array([[20, 20, 1],[0, 40, 1]])
    #pose = np.array([-1,0,0])
    time_segments = [5]

    time_optim_flag = False
    corr_const_flag = True
    corridor_radius = 0.02
    num_points_corr = 4.0
    corr_length = 1.0
    time_corr = np.array([0.2])
    print(time_corr.shape[0])
    k_t = 50
    step_der = 0.01
    single_segment = True
    traj_test = Trajectory_generation(deg,inital_der,final_der,waypoint,pose,time_segments,time_optim_flag,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,k_t,step_der,single_segment,specify_final_der)

    #traj_test.create_waypoint()
    #traj_test.A_corridor_constraint()
    sol = traj_test.solve()
    print(sol)
    #print(traj_test.time_segments)

    print(time.time()-start_time)
