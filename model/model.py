import sys
import open3d as o3d
import numpy as np

VOXEL_SIZE =  0.005

pcd = o3d.io.read_point_cloud("plane_final.ply") 
# o3d.visualization.draw_geometries([pcd])

pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)

o3d.visualization.draw_geometries([pcd])
# o3d.visualization.draw([pcd])
