import sys
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("plane_final.ply")
# o3d.visualization.draw([pcd])

radius = 0.1  
max_nn = 30  
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))

alpha = 0.01
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()

o3d.visualization.draw_geometries([mesh])
