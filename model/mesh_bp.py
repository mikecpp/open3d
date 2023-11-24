import sys
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("plane_final.ply")
# o3d.visualization.draw([pcd])

radius = 0.1  
max_nn = 30  
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))

radii = [0.005, 0.01, 0.02, 0.04] 
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

o3d.visualization.draw_geometries([mesh])
