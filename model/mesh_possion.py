import sys
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("plane_final.ply")
# o3d.visualization.draw([pcd])

radius = 0.1  
max_nn = 30
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)
mesh.compute_vertex_normals()

o3d.visualization.draw([mesh])
