import sys
import open3d as o3d
import numpy as np

VOXEL_SIZE =  0.005

pcd = o3d.io.read_point_cloud("plane_final.ply") 
# o3d.visualization.draw_geometries([pcd])

pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)

diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
camera = [0, 0, diameter]
radius = diameter * 10000
_, pt_map = pcd.hidden_point_removal(camera, radius)
pcd = pcd.select_by_index(pt_map)

o3d.visualization.draw_geometries([pcd])
# o3d.visualization.draw([pcd])
