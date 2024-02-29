import open3d as o3d
import numpy as np

VOXEL_SIZE =  0.001

pcd = o3d.io.read_point_cloud("box.ply") 

pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0) 

points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.30]
points = points[points[:, 0] >= -0.30]
pcd.points = o3d.utility.Vector3dVector(points)
# o3d.visualization.draw_geometries([pcd]) 

# table plane 
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=10000)
[a, b, c, d1] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d1:.3f} = 0")
table_pcd = pcd.select_by_index(inliers)
table_pcd.paint_uniform_color([1.0, 0, 0]) 

other_pcd = pcd.select_by_index(inliers, invert=True)

# box plane 
plane_model, inliers = other_pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=10000)
[a, b, c, d2] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d2:.3f} = 0")

depth = d1 - d2 
print(f"Box depth: {depth*1000:.0f} mm")
box_pcd = other_pcd.select_by_index(inliers)

points = np.asarray(box_pcd.points) 
points[-1, 2] -= depth
box_pcd.points = o3d.utility.Vector3dVector(points)

box_pcd.paint_uniform_color([0, 0, 1.0]) 

bbox = box_pcd.get_axis_aligned_bounding_box() 
bbox.color = (0, 1, 0) 
print("offset:", bbox.max_bound - bbox.min_bound) 

o3d.visualization.draw_geometries([pcd, bbox]) 
