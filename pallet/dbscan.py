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

o3d.visualization.draw_geometries([pcd])

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(eps=0.1, min_points=10, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")

colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

o3d.visualization.draw_geometries([pcd])
