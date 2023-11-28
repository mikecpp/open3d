import sys
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("plane_final.ply")
pcd.paint_uniform_color([0.5, 0.5, 0.5])
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

# [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 100)
# np.asarray(pcd.colors)[idx[1:], :] = [1, 0, 0]

[k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1000], 0.2)
np.asarray(pcd.colors)[idx[1:], :] = [1, 0, 0]

o3d.visualization.draw_geometries([pcd])
