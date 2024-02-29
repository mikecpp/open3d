import open3d as o3d
import numpy as np
import open3d.visualization as visual
from text_3d import text_3d

VOXEL_SIZE =  0.001

### Read original point cloud 
pcd = o3d.io.read_point_cloud("box.ply") 

### Down sample with voxel size 1mm 
pcd = pcd.voxel_down_sample(voxel_size = VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0) 
o3d.visualization.draw_geometries([pcd]) 

### Remove the non-Table objects 
points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.30]
points = points[points[:, 0] >= -0.30] 
pcd.points = o3d.utility.Vector3dVector(points)
# o3d.visualization.draw_geometries([pcd]) 

### Get table top plane 
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=10000)
[a, b, c, d1] = plane_model
table_pcd = pcd.select_by_index(inliers)
table_pcd.paint_uniform_color([1.0, 0, 0]) 
other_pcd = pcd.select_by_index(inliers, invert=True)

### Get box top plane 
plane_model, inliers = other_pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=10000)
[a, b, c, d2] = plane_model

### Calculate box height from 2 planes
depth = d1 - d2 
box_pcd = other_pcd.select_by_index(inliers)

### Pick up any box point plus depth 
points = np.asarray(box_pcd.points) 
points[-100, 2] -= (depth - .005)
box_pcd.points = o3d.utility.Vector3dVector(points)
box_pcd.paint_uniform_color([0, 0, 1.0]) 

### Get box bounding box as box length, width & height 
bbox = box_pcd.get_axis_aligned_bounding_box() 
bbox.color = (0, 1, 0) 
bound = bbox.max_bound - bbox.min_bound

msg = f"W: {bound[0]*1000:.0f}mm, L: {bound[1]*1000:.0f}mm, H: {bound[2]*1000:.0f}mm"
print(msg)

pcd_text = text_3d(msg, pos=bbox.get_box_points()[6], font_size=15)

o3d.visualization.draw_geometries([pcd, bbox, pcd_text],
                                  zoom = 0.20,
                                  front  = [ 0.23, -0.23, 0.95 ],
                                  lookat = [ 4.20e-05, -0.00, -1.54 ],
                                  up     = [ -0.95, 0.16, 0.27 ])
