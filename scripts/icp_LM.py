# -*- coding: utf-8 -*-
"""
Created on Thu Mar 10 12:20:15 2022

@author: IntekPlus
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
import utils


class icp_LM():
    def __init__(self, grid_size, distance_threshold, iterations, mode, animate, model, scene):
        self.dist = distance_threshold
        
         #scale pointclouds
        self.allpts = np.concatenate((model.points, scene.points), axis=0)
        self.min, self.max = np.min(self.allpts, axis=0), np.max(self.allpts, axis=0)
        
        # get model pcd
        self.model = model
        self.model = self.model.voxel_down_sample(grid_size)
        self.tree = o3d.geometry.KDTreeFlann(self.model)
        self.norm_param = o3d.geometry.KDTreeSearchParamKNN(15)
        self.model.estimate_normals(self.norm_param)
        self.model.orient_normals_consistent_tangent_plane(k=15)
        self.normals = np.asarray(self.model.normals)
        
        # get scene  
        self.scene = scene
        self.scene = self.scene.voxel_down_sample(grid_size)
        
        #outputs
        self.rotation = np.eye(3)
        self.translation = np.zeros((3,1))
        self.euler = np.zeros(6)
        self.transformedCloud = o3d.geometry.PointCloud()

        #set static and dynamic starting clouds
        self.staticPointCloud = np.asarray(self.model.points).transpose(1,0)
        self.dynamicPointCloud = np.asarray(self.scene.points).transpose(1,0)
        self.error_init = np.linalg.norm(self.staticPointCloud.mean(axis=1) - self.dynamicPointCloud.mean(axis=1))
        
        #get sizes
        self.numDynamicPoints = self.dynamicPointCloud.shape[1]
        self.numStaticPoints = self.staticPointCloud.shape[1]
        
        self.weights = np.ones(self.numDynamicPoints)
        
        #parameters
        self.maxIterations = iterations
        self.numRandomSamples = min(self.numDynamicPoints, self.numStaticPoints)
        self.eps = 1e-3
        self.cost = [1e09]
        self.lmda = 1e-2
        
        # Create figure display
        self.fig = plt.figure()
        self.ax = plt.axes(xlim = [self.min[0],self.max[0]], projection='3d')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.ax.set_title("point-to-{}-LM".format(mode))
        utils.axisEqual3D(self.ax)
        self.static = self.ax.scatter(self.staticPointCloud[0,:], self.staticPointCloud[1,:], self.staticPointCloud[2,:], marker='.',color = "blue")
        self.ax.scatter(self.dynamicPointCloud[0,:], self.dynamicPointCloud[1,:], self.dynamicPointCloud[2,:], marker='.', color = "red")
        self.dynamic = self.ax.scatter(self.dynamicPointCloud[0,:], self.dynamicPointCloud[1,:], self.dynamicPointCloud[2,:], marker='.', color = "green")
        self.label = self.ax.text(-0.5, -0.5, 50, "Step = 0  Error: %.2d"%(self.error_init))
        self.lines = []
        self.projs = []
        
        '''
        for i in range(self.numRandomSamples):
            line, = self.ax.plot3D([self.staticPointCloud[0,i], self.dynamicPointCloud[0,i]],[self.staticPointCloud[1,i], self.dynamicPointCloud[1,i]],[self.staticPointCloud[2,i], self.dynamicPointCloud[2,i]], color = 'c')
            self.lines.append(line)
            
        for i in range(self.numRandomSamples):
            proj, = self.ax.plot3D([self.staticPointCloud[0,i], self.dynamicPointCloud[0,i]],[self.staticPointCloud[1,i], self.dynamicPointCloud[1,i]],[self.staticPointCloud[2,i], self.dynamicPointCloud[2,i]], color = 'k')
            self.projs.append(proj)
        '''
        
        if mode == "point":
            if not animate:
                iter = 0
                while iter < self.maxIterations:
                    self.update_point_to_point(iter)
                    iter +=1
            else:
                self.func = FuncAnimation(self.fig, self.update_point_to_point, frames = self.maxIterations, interval = 1, repeat = False)
                plt.show()      
        elif mode == "plane":
            if not animate:
                iter = 0
                while iter < self.maxIterations:
                    self.update_point_to_plane(iter)
                    iter +=1
            else:
                self.func = FuncAnimation(self.fig, self.update_point_to_plane, frames = self.maxIterations, interval = 1, repeat = False)
                plt.show()
        else:
            raise Exception("Invalid Mode")
        
  
    def get_correspondences(self):
        #sample
        rand = random.choices(range(self.numDynamicPoints), k = self.numRandomSamples)
        p = self.dynamicPointCloud[:,rand]
        x = np.zeros_like(p)
        n = np.zeros_like(p)
        
        for i in range(self.numRandomSamples):
            _ , idx, dsq = self.tree.search_knn_vector_3d(p[:,i], 1)
            x[:,i] = self.staticPointCloud[:, idx[0]]
            n[:,i] = self.normals[idx[0], :]
            
        return p, x, n
    
    def update_point_to_point(self, framenumber):
        
        P, X, _ = self.get_correspondences()
        #######################################################################
        ###########################Levenberg Marquardt################################
        
        #Jacobian
        H = np.zeros((6,6))
        b = np.zeros((6,1))
        Rot, trans = self.eulerToRot(self.euler)
        chi = 0
        
        for i in range(self.numRandomSamples):
            p,x = P[:,i], X[:,i]
            Rot_p = Rot@p
            e = (Rot_p + trans).reshape(3) - x
            
            J = self.Jacobian_point(Rot_p)
            H += J.T@J 
            b += J.T@(e.reshape(3,1))
            chi += np.linalg.norm(e)
            
            #print(i, e)
            #f = p[:,i] - e.reshape(3)
            #self.lines[i].set_data_3d([x[0,i], p[0,i]],[x[1,i], p[1,i]],[x[2,i], p[2,i]])
            #self.projs[i].set_data_3d([f[0], p[0,i]],[f[1], p[1,i]],[f[2], p[2,i]])
        
        H = self.lmda*H*np.eye(6)
        update = - np.linalg.inv(H)@b
        
        #LM step
        self.euler_new = self.euler + update.reshape(6)
        
        Rot_new, trans_new = self.eulerToRot(self.euler_new)
        e_new = Rot_new@P + trans_new.reshape(3,1) - X
        chi_new = np.linalg.norm(e_new, axis=0).sum()
        
        if chi_new > chi:
            self.lmda *= 10
            
        else:
            self.euler = self.euler_new
            self.lmda /= 10
            
        #######################################################################
        #######################################################################        
        
        #update cloud for next iteration
        rotation, translation = self.eulerToRot(self.euler)
        self.dynamicPointCloud = rotation@(self.dynamicPointCloud) + translation.reshape(3,1)
        
        #self.rotation = rotation@self.rotation
        #self.translation = rotation@self.translation + translation.reshape(3,1)
        
        #update plots
        self.label.set_text("Step = %.2d  Error = %.2f"%(framenumber, chi/self.numRandomSamples))
        self.dynamic._offsets3d = (P[0,:], P[1,:], P[2,:])
        #self.static._offsets3d = (x[0,:], x[1,:], x[2,:])
        
    def update_point_to_plane(self, framenumber):
        
        P, X, N = self.get_correspondences()
        #o3d.visualization.draw_geometries([self.model, self.scene])
        #######################################################################
        ###########################Levenberg Marquardt################################
        
        #Jacobian
        H = np.zeros((6,6))
        b = np.zeros((6,1))
        Rot, trans = self.eulerToRot(self.euler)
        chi = 0
        
        for i in range(self.numRandomSamples):
            p,x,n = P[:,i], X[:,i], N[:,i]
            Rot_p = Rot@p
            e = (Rot_p.reshape(3) + trans - x).dot(n)
            J = self.Jacobian_plane(Rot_p, n)
            H += J.T@J 
            b += J.T*e
            chi += np.linalg.norm(e)
            
            #f = p[:,i] + e*n[:,i].reshape(3) #2*normal.reshape(3) #
            #print(mag, e, f.shape)
            #self.lines[i].set_data_3d([x[0,i], p[0,i]],[x[1,i], p[1,i]],[x[2,i], p[2,i]])
            #self.projs[i].set_data_3d([f[0], p[0,i]],[f[1], p[1,i]],[f[2], p[2,i]])
            
        H += self.lmda*H*np.eye(6)
        update = - np.linalg.inv(H)@b
        
        #LM step
        self.euler_new = self.euler + update.reshape(6)
        
        Rot_new, trans_new = self.eulerToRot(self.euler_new)
        
        chi_new=0
        for i in range(self.numRandomSamples):
            e_new = ((Rot_new@P[:,i]).reshape(3) + trans_new - X[:,i]).dot(N[:,i])
            chi_new += np.linalg.norm(e_new)
        
        if chi_new > chi:
            self.lmda *= 10
            
        else:
            self.euler = self.euler_new
            self.lmda /= 10
           
        #######################################################################
        #######################################################################        
        
        #update cloud for next iteration
        rotation, translation = self.eulerToRot(self.euler)
        
        self.dynamicPointCloud = rotation@(self.dynamicPointCloud) + translation.reshape(3,1)
        
        #self.rotation = rotation@self.rotation
        #self.translation = rotation@self.translation + translation.reshape(3,1)
        #self.euler = self.rotToEuler(self.rotation, self.translation)
        
        #update plots
        self.label.set_text("Step = %.2d  Error = %.2f"%(framenumber, chi/self.numRandomSamples))
        self.dynamic._offsets3d = (P[0,:], P[1,:], P[2,:])
        #self.static._offsets3d = (x[0,:], x[1,:], x[2,:])
            
    
    def Jacobian_point(self, Rot_p):
        J = np.zeros((3,6))
        J[:, 3:] = np.eye(3)
        J[:,0] = np.cross(Rot_p, np.array([1, 0, 0]))
        J[:,1] = np.cross(Rot_p, np.array([0, 1, 0]))
        J[:,2] = np.cross(Rot_p, np.array([0, 0, 1]))
        return J
    
    def Jacobian_plane(self, Rot_p, n):
        J = np.zeros((1,6))
        J[:, 3:] = n
        J[:,:3] = -np.cross(Rot_p, n) 
        return J
    
    def rotToEuler(self, R, t):
        x,y,z = t[0,0], t[1,0], t[2,0]
        cs = np.linalg.norm(R[:2,0])
        if cs < 1e-16:
            alpha = np.arctan2(-R[1,2], R[1,1])
            beta = np.arctan2(-R[1,2], cs)
            gamma = 0
        else:
            alpha = np.arctan2(R[2,1], R[2,2])
            beta = np.arctan2(-R[2,0], cs)
            gamma = np.arctan2(R[1,0], R[0,0])
        
        return np.array([alpha, beta, gamma, x, y, z])
  
    def eulerToRot(self, euler):
        alpha, beta, gamma, x, y, z = euler[0], euler[1], euler[2], euler[3], euler[4], euler[5]

        Rot_z = np.array([np.cos(gamma), np.sin(gamma), 0, -np.sin(gamma), np.cos(gamma), 0, 0, 0, 1]).reshape(3, 3)
        Rot_y = np.array([np.cos(beta), 0, -np.sin(beta), 0, 1, 0, np.sin(beta), 0, np.cos(beta)]).reshape(3, 3)
        Rot_x = np.array([1, 0, 0, 0, np.cos(alpha), np.sin(alpha), 0, -np.sin(alpha),  np.cos(alpha)]).reshape(3, 3)
        Rot = Rot_z@Rot_y@Rot_x
        return Rot, np.array([x,y,z])

