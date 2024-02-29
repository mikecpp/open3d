import open3d as o3d
import numpy as np

VOXEL_SIZE =  0.001

box = o3d.io.read_point_cloud("box.ply") 
pcd = box

pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0) 

points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.25]
points = points[points[:, 0] >= -0.30]
points = points[points[:, 2] >= -1.60]
box.points = o3d.utility.Vector3dVector(points)

bbox = box.get_axis_aligned_bounding_box() 
bbox.color = (1, 0, 0) 

print("offset:", bbox.max_bound - bbox.min_bound) 
# o3d.visualization.draw_geometries([pcd, bbox]) 
# o3d.visualization.draw_geometries([box, bbox]) 
