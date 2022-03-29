# -*- coding: utf-8 -*-
"""
Created on Thu Mar 10 12:20:15 2022

@author: IntekPlus
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def axisEqual3D(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


def Rot3D(vecin, alpha, beta, gamma, x, y, z):
    s = np.pi/180
    alpha = alpha*s
    beta = beta*s
    gamma = gamma*s
    Rot_z = np.array([np.cos(gamma), np.sin(gamma), 0, -np.sin(gamma), np.cos(gamma), 0, 0, 0, 1]).reshape(3, 3)
    Rot_y = np.array([np.cos(beta), 0, -np.sin(beta), 0, 1, 0, np.sin(beta), 0, np.cos(beta)]).reshape(3, 3)
    Rot_x = np.array([1, 0, 0, 0, np.cos(alpha), np.sin(alpha), 0, -np.sin(alpha),  np.cos(alpha)]).reshape(3, 3)
    Rot = Rot_z@Rot_y@Rot_x
    return (Rot@(vecin.transpose(1, 0))).transpose(1, 0) + np.array([x, y, z])        


def NanInfScale(cloud, scale):
    points = scale * np.asarray(cloud.points)
    #points = points[~np.isnan(points)]
    #points = points[~np.isinf(points)]
    #points = points.reshape(-1,3)
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(points)
    return out

