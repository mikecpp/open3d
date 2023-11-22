import sys
import open3d as o3d
import numpy as np

THRESHOLD  = -0.45
VOXEL_SIZE =  0.01

# Load PLY files
pcd = o3d.io.read_point_cloud("pallet_01.ply") 

''' 
# Down sample and filter 
pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
# pcd = pcd.uniform_down_sample(every_k_points=5)
o3d.io.write_point_cloud("pallet_down.ply", pcd, write_ascii=True)

# Remove outlier 
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
o3d.io.write_point_cloud("pallet_remove.ply", pcd, write_ascii=True)

# Remove reflection 
points = np.asarray(pcd.points) 
pcd.points = o3d.utility.Vector3dVector(points[points[:, 1] >= THRESHOLD])
o3d.io.write_point_cloud("pallet_final.ply", pcd, write_ascii=True)
'''

pcd.estimate_normals()

radii = [0.01, 0.02, 0.04, 0.08]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])

# Show final point cloud 
# o3d.visualization.draw([pcd])
