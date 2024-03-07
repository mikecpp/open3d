import open3d as o3d
import numpy as np
import sys
import time 

VOXEL_SIZE =  0.001

pcd = o3d.io.read_point_cloud(sys.argv[1]) 
# o3d.visualization.draw_geometries([pcd]) 

pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0) 
# o3d.visualization.draw_geometries([pcd]) 

points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.30]
points = points[points[:, 0] >= -0.30]
pcd.points = o3d.utility.Vector3dVector(points)
# o3d.visualization.draw_geometries([pcd]) 

time_start = time.time() 
# table plane 
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d1] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d1:.3f} = 0")
table_pcd = pcd.select_by_index(inliers)
table_pcd.paint_uniform_color([1.0, 0, 0]) 

other_pcd = pcd.select_by_index(inliers, invert=True)

# box plane 
plane_model, inliers = other_pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=1000)
[a, b, c, d2] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d2:.3f} = 0")

depth = d1 - d2 
# print(f"Box depth: {depth*1000:.0f} mm")
box_pcd = other_pcd.select_by_index(inliers)

points = np.asarray(box_pcd.points) 
points[-100, 2] -= (depth - .005)
box_pcd.points = o3d.utility.Vector3dVector(points)

box_pcd.paint_uniform_color([0, 0, 1.0]) 

bbox = box_pcd.get_axis_aligned_bounding_box() 
bbox.color = (0, 1, 0) 
bound = bbox.get_max_bound() - bbox.get_min_bound()

time_end = time.time()

print(f"width: {bound[0]*1000:.0f}mm, length: {bound[1]*1000:.0f}mm, height: {bound[2]*1000:.0f}mm")
print(f"time elapse: {(time_end - time_start)*1000:.0f} ms")
print("-----------------------------------------------")

# o3d.visualization.draw_geometries([pcd, bbox]) 
