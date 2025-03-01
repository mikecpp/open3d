import open3d as o3d
import numpy as np
import time

VOXEL_SIZE     = 0.001
NB_NEIGHBORS   = 40
STD_RATIO      = 2.0

TABLE_LEFT     = -0.30
TABLE_RIGHT    =  0.30

TABLE_DISTANCE = 0.01 
RANSAC_N       = 3
NUM_ITERS      = 1000

BOX_DISTANCE   = 0.015 

### Read point cloud 
pcd = o3d.io.read_point_cloud("field_0_500us.ply") 
# o3d.visualization.draw_geometries([pcd])

### Remove the non-Table objects 
points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  TABLE_RIGHT] 
points = points[points[:, 0] >=  TABLE_LEFT] 
pcd.points = o3d.utility.Vector3dVector(points) 
# o3d.visualization.draw_geometries([pcd])

### Get table top plane 
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d1] = plane_model
table_pcd = pcd.select_by_index(inliers)
table_pcd.paint_uniform_color([1.0, 0, 0]) 
other_pcd = pcd.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([table_pcd])
# o3d.visualization.draw_geometries([other_pcd])

### Get box top plane 
plane_model, inliers = other_pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=1000)
[a, b, c, d2] = plane_model
box_pcd = other_pcd.select_by_index(inliers)
# o3d.visualization.draw_geometries([box_pcd])

### Calculate box height from 2 planes
depth = d1 - d2 

### Pick up any box point plus depth 
points = np.asarray(box_pcd.points) 
points[-100, 2] -= (depth - .005)
box_pcd.points = o3d.utility.Vector3dVector(points)
box_pcd.paint_uniform_color([0, 0, 1.0]) 

### Get box bounding box as box length, width & height 
bbox = box_pcd.get_axis_aligned_bounding_box() 
# bbox = box_pcd.get_oriented_bounding_box() 
bbox.color = (1, 0, 0) 
bound = bbox.get_max_bound() - bbox.get_min_bound()

msg = f"W: {bound[0]*1000:.0f}mm, L: {bound[1]*1000:.0f}mm, H: {bound[2]*1000:.0f}mm"
print(msg)

# pcd_text = text_3d(msg, pos=bbox.get_box_points()[6], font_size=15)

'''
o3d.visualization.draw_geometries([pcd, bbox, pcd_text],
                                  zoom = 0.50,
                                  front  = [ 0.23, -0.23, 0.95 ],
                                  lookat = [ 4.20e-05, -0.00, -1.54 ],
                                  up     = [ -0.95, 0.16, 0.27 ])
'''
