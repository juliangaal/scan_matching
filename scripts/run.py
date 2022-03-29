# -*- coding: utf-8 -*-
"""
Created on Thu Mar 10 12:20:15 2022

@author: IntekPlus
"""

import numpy as np
import open3d as o3d
import utils
from icp_GN import icp_GN
from icp_LM import icp_LM

model = o3d.io.read_point_cloud("../data/bun000_UnStructured.pcd")
scene = o3d.io.read_point_cloud("../data/bun045_UnStructured.pcd")

model = utils.NanInfScale(model, 1000)
scene = utils.NanInfScale(scene, 1000)

model.points = o3d.utility.Vector3dVector(model.points - np.mean(model.points, axis=0)) 
scene.points = o3d.utility.Vector3dVector(scene.points - np.mean(scene.points, axis=0))
scene.points = o3d.utility.Vector3dVector(utils.Rot3D(np.asarray(scene.points), -30, 20, 0, 10, 0, 0))

# args: grid_size, distance threshold, iterations, error mode, model, scene
icp = icp_GN(4, 1, 20, "plane", True, model, scene)
icp = icp_LM(4, 1, 20, "plane", True, model, scene)
