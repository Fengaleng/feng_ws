#!/usr/bin/env python
import numpy as np
import trajectory_generation
from scipy.optimize import minimize
from scipy.optimize import fmin_cg
from scipy.linalg import block_diag
import time
import sys

np.set_printoptions(threshold=sys.maxsize)


class Trajectory_generation_time:
    def __init__(self,deg,inital_der,final_der,waypoint,pose,time_segments,time_optim_flag,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,k_t,step_der,single_segment,specify_final_der):
        #self.traj_test = trajectory_generation.Trajectory_generation(deg,inital_der,final_der,waypoint,pose,time_segments,time_optim_flag,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,k_t,step_der,single_segment)
        self.kt = k_t
        self.initial_time = time_segments
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
        size = int(self.deg+(self.deg+2)*self.num_segments)
        size2 = int(self.num_coeff*self.num_segments)
        self.A_eq = np.zeros((size,size2))
        self.b_eq_x = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
        self.b_eq_y = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
        self.b_eq_z = np.zeros((int(self.deg+(self.deg+2)*self.num_segments),1))
        self.single_segment = single_segment
        self.specify_final_der = specify_final_der
    def cost_function(self,time_seg):
        time_seg = np.absolute(time_seg)
        #for i in range(0,time_seg.shape[0]):
        #    if time_seg[i] < 0.05:
        #        time_seg[i] = 0.05
        #print(time_seg[i])

        #print(time_seg)
        #print(time_seg)
        self.quad_cost = trajectory_generation.Trajectory_generation(self.deg,self.inital_der,self.final_der,self.waypoint,self.pose,time_seg,self.time_optim_flag,self.corr_const_flag,self.corridor_radius,self.num_points_corr,self.corr_length,self.time_corr,self.k_t,self.step_der,self.single_segment,self.specify_final_der)
        p1,p2,p3 = self.quad_cost.solvemat_time()
        P = self.quad_cost.Q_x
        #size = x.shape[0]
        #self.x = x
        #cost = np.dot(np.dot(p1.transpose(),P),p1)+np.dot(np.dot(p2.transpose(),P),p2)+np.dot(np.dot(p3.transpose(),P),p3)+self.k_t*np.sum(time_seg)
        cost = np.dot(np.dot(p1.transpose(),P),p1)+np.dot(np.dot(p2.transpose(),P),p2)+np.dot(np.dot(p3.transpose(),P),p3)
        print(cost)
        #print(time_seg)


        return cost

    #def first_order_grad(self,time)

    def optimal_time(self):
        pass

    def optimize_time(self):
        traj_obj = trajectory_generation.Trajectory_generation(self.deg,self.inital_der,self.final_der,self.waypoint,self.pose,self.time_segments,self.time_optim_flag,self.corr_const_flag,self.corridor_radius,self.num_points_corr,self.corr_length,self.time_corr,self.k_t,self.step_der,self.single_segment,self.specify_final_der)
        traj_obj.create_waypoint()
        self.waypoint = traj_obj.waypoint
        #step = 0.00001*np.ones(traj_obj.time_segments.shape)
        step = 1e-6
        gradient_val = (self.cost_function(traj_obj.time_segments+step)-self.cost_function(traj_obj.time_segments))/step
        gradient_val_2 = (self.cost_function(traj_obj.time_segments+step)-2*self.cost_function(traj_obj.time_segments)+self.cost_function(traj_obj.time_segments-step))/(step*step)
        print(gradient_val)
        print(gradient_val_2)
        bounds = np.zeros((traj_obj.time_segments.shape[0],2))
        for i in range(0,traj_obj.time_segments.shape[0]):
            bounds[i,0]= 0.1
            bounds[i,1]= 10

        print(traj_obj.time_segments)

        jac_test = np.identity(12)
        print(jac_test)

        eq_cons = {'type':'eq', 'fun' : lambda x: np.array(30-x[0]-x[1]-x[2]-x[3]-x[4]-x[5]-x[6]-x[7]-x[8]-x[9]-x[10]-x[11]-x[12]-x[13])}
        #ineq_cons = {'type':'ineq', 'fun' : lambda x: np.array([x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],x[11]])}
        bnds = ((0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None),(0.1,None))
        print(len(bnds))
        print(traj_obj.time_segments.shape)
        res = minimize(self.cost_function,x0 = traj_obj.time_segments,bounds = bnds,constraints=eq_cons,options={'disp': True})
        #res = minimize(self.cost_function,x0 = traj_obj.time_segments,options={'disp': True})
        print(res)
        #x0 = np.array([ 5.0,  3.0,  1.0,  4.0,  7.0])
        #res2 = fmin_cg(self.cost_function,x0 = traj_obj.time_segments)
        #print(np.absolute(res.x))
        #print(res2)
        #print(self.x)



if __name__ == '__main__':
    start_time = time.time()
    deg = 4
    inital_der = np.zeros((deg,4))
    inital_der[0,0] = 0
    final_der = np.zeros((deg,4))
    waypoint = np.array([[0,0,0],[20, 20, 0],[0, 40, 0],[-20,20,0 ],[0,0,0],[20, 20, 0],[0, 40, 0],[-20,20,0 ]])
    #waypoint = np.array([[0,0,0],[20, 20, 0]])
    pose = np.array([[0,1,0],[-1,0,0],[0,-1,0],[1,0,0],[0,1,0],[-1,0,0],[0,-1,0],[1,0,0]])
    #time_segments = np.array([[10],[10],[10],[10],[10],[10],[10],[10]])
    #waypoint = np.array([[20, 20, 1],[0, 40, 1]])
    #pose = np.array([-1,0,0])
    #time_segments = [5]

    time_optim_flag = False
    corr_const_flag = True
    corridor_radius = 0.2
    num_points_corr = 4.0
    corr_length = 2.0
    avg_spd = 7.0
    time_corr = corr_length/(avg_spd)
    k_t = 50
    step_der = 0.01
    single_segment = True
    specify_final_der = False

    #waypoint = np.array([[17.1813,3.4022,6.3065],[16.7877,39.441,6.7625],[2.0897,28.4731,2.5465],[1.7848,7.7087,1.9937],[-7.3087,-12.1367,3.2299],[-8.3056,-28.4834,2.6425],[-1.4695,-31.7464,2.1031],[7.1432,-28.7472,2.5675],[7.2498,-12.7062,2.5465],[-12.6908,9.444,2.0199],[-10.0486,30.6232,2.8979]])
    #pose = np.array([[0,1,0],[-0.707,0.707,0],[0,-1,0],[0,-1,0],[0,-1,0],[0.707,-0.707,0],[1,0,0],[0.707,0.707,0],[0,1,0],[0.0,1,0],[0,1,0]])
    time_segments = np.zeros((waypoint.shape[0])-1)
    for i in range(0,waypoint.shape[0]-1):
        time_segments[i] = np.linalg.norm(waypoint[i+1,0:3]-waypoint[i,0:3])/avg_spd






    traj_time_optim = Trajectory_generation_time(deg,inital_der,final_der,waypoint,pose,time_segments,time_optim_flag,corr_const_flag,corridor_radius,num_points_corr,corr_length,time_corr,k_t,step_der,single_segment,specify_final_der)
    traj_time_optim.optimize_time()
    print(time.time()-start_time)
    #print(traj_time_optim.waypoint)
