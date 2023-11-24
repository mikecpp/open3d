import sys
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("final.ply")
o3d.visualization.draw([pcd])

radius = 0.1  
max_nn = 30  
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))

### Ball Pivoting
radii = [0.005, 0.01, 0.02, 0.04] 
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

### Alpha 
# alpha = 0.01
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
# mesh.compute_vertex_normals()

### Poisson
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)

o3d.visualization.draw_geometries([mesh])
