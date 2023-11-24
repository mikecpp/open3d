import sys
import open3d as o3d
import numpy as np

THRESHOLD  = -0.45
VOXEL_SIZE =  0.02

# Load PLY files
pcd = o3d.io.read_point_cloud(sys.argv[1]) 
# o3d.visualization.draw([pcd])

# Down sample
pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)

# Remove outlier
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
points = np.asarray(pcd.points) 
pcd.points = o3d.utility.Vector3dVector(points[points[:, 1] >= THRESHOLD])

# Create mesh
mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0) 
mesh_box.translate((-0.5, -0.45, -3.0))

# pcd_box = mesh_box.sample_points_uniformly(number_of_points=10000) 
# pcd_box.paint_uniform_color([1.0, 0, 0])
o3d.visualization.draw([pcd])
o3d.visualization.draw([pcd, mesh_box])
