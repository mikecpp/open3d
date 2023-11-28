import sys
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

THRESHOLD  = -0.45
VOXEL_SIZE =  0.02

pcd = o3d.io.read_point_cloud("pallet_01.ply") 
pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
points = np.asarray(pcd.points) 
pcd.points = o3d.utility.Vector3dVector(points[points[:, 1] >= THRESHOLD])
# o3d.visualization.draw_geometries([pcd])

plane_model, inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)

[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])

outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw([inlier_cloud, outlier_cloud])
