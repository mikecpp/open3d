import sys
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("plane_final.ply")

mesh = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
# mesh.compute_vertex_normals()

o3d.visualization.draw_geometries([mesh])
