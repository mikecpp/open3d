import open3d as o3d
import numpy as np

VOXEL_SIZE =  0.001

### Read original point cloud 
pcd = o3d.io.read_point_cloud("field.ply") 

### Down sample with voxel size 1mm 
pcd = pcd.voxel_down_sample(voxel_size = VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0) 
# o3d.visualization.draw_geometries([pcd]) 

### Remove the non-Table objects 
points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.30]
points = points[points[:, 0] >= -0.30] 
pcd.points = o3d.utility.Vector3dVector(points)
# o3d.visualization.draw_geometries([pcd]) 

o3d.io.write_point_cloud("box.ply", pcd, write_ascii = True)
