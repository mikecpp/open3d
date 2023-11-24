import sys
import open3d as o3d
import numpy as np
import copy

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp]) 
                                      
# Load PLY files
source = o3d.io.read_point_cloud("final_01.ply") 
target = o3d.io.read_point_cloud("final_02.ply") 
o3d.visualization.draw([source, target])

threshold = 0.02

reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, np.identity(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

draw_registration_result(source, target, reg_p2p.transformation)
