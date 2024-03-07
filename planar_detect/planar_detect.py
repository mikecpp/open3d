import open3d as o3d
import numpy as np
import sys
import time 

VOXEL_SIZE =  0.001

pcd = o3d.io.read_point_cloud(sys.argv[1]) 
# o3d.visualization.draw_geometries([pcd]) 

pcd = pcd.voxel_down_sample(voxel_size = VOXEL_SIZE) 
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors = 40, std_ratio = 2.0) 
# o3d.visualization.draw_geometries([pcd]) 

points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.30] 
points = points[points[:, 0] >= -0.30] 
pcd.points = o3d.utility.Vector3dVector(points) 
# o3d.visualization.draw_geometries([pcd]) 

pcd.estimate_normals() 

time_start = time.time() 
oboxes = pcd.detect_planar_patches()

width   = oboxes[0].extent[0] * 1000
length  = oboxes[0].extent[1] * 1000
height  = abs(oboxes[0].center[2] - oboxes[1].center[2]) * 1000 
time_end = time.time()

print(f"width: {width:.0f} mm, length: {length:.0f} mm, height: {height:.0f} mm") 
print(f"time elapse: {(time_end - time_start)*1000:.0f} ms")
print("-----------------------------------------------")

# o3d.visualization.draw_geometries([oboxes[0], oboxes[1]]) 
