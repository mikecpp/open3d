import sys
import open3d as o3d
import numpy as np

THRESHOLD  = -0.45
VOXEL_SIZE =  0.005

# Load PLY files
pcd = o3d.io.read_point_cloud("model_01.ply") 
o3d.visualization.draw_geometries([pcd])

# Down sample and filter 
pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
# o3d.io.write_point_cloud("pallet_down.ply", pcd, write_ascii=True)

# Remove outlier 
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
# o3d.io.write_point_cloud("pallet_remove.ply", pcd, write_ascii=True)

# Remove reflection 
points = np.asarray(pcd.points) 
pcd.points = o3d.utility.Vector3dVector(points[points[:, 1] >= THRESHOLD])

# Show final point cloud 
o3d.visualization.draw_geometries([pcd])
# o3d.visualization.draw([pcd])
