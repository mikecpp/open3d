import open3d as o3d
import numpy as np
import open3d.visualization as visual
from text_3d import text_3d

VOXEL_SIZE =  0.001

### Read original point cloud 
pcd = o3d.io.read_point_cloud("box_1000us.ply") 

### Down sample with voxel size 1mm 
pcd = pcd.voxel_down_sample(voxel_size = VOXEL_SIZE) 
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0) 
# o3d.visualization.draw_geometries([pcd]) 

### Remove the non-Table objects 
points = np.asarray(pcd.points) 
points = points[points[:, 0] <=  0.30] 
points = points[points[:, 0] >= -0.30] 
pcd.points = o3d.utility.Vector3dVector(points) 
# o3d.visualization.draw_geometries([pcd]) 

### Get table top plane 
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d1] = plane_model
pcd_table = pcd.select_by_index(inliers)
pcd_table.paint_uniform_color([1.0, 0, 0]) 
other_pcd = pcd.select_by_index(inliers, invert=True)

### Get box top plane 
plane_model, inliers = other_pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=1000)
[a, b, c, d2] = plane_model

### Calculate box height from 2 planes
depth = d1 - d2 
pcd_box = other_pcd.select_by_index(inliers)

### Pick up any box point plus depth 
points = np.asarray(pcd_box.points) 
points[-100, 2] -= (depth - .005)
pcd_box.points = o3d.utility.Vector3dVector(points)
pcd_box.paint_uniform_color([0, 0, 1.0]) 

### Get box bounding box as box length, width & height 
bbox_box = pcd_box.get_axis_aligned_bounding_box() 
bbox_box.color = (1, 0, 0) 
bound_box = bbox_box.get_max_bound() - bbox_box.get_min_bound()

### Get table bounding box 
bbox_table = pcd_table.get_axis_aligned_bounding_box() 
bbox_table.color = (0, 1, 0) 
bound_table = bbox_table.get_max_bound() - bbox_table.get_min_bound()
mesh_table = o3d.geometry.TriangleMesh.create_box(width = bound_table[0], height = bound_table[1], depth = bound_table[2]) 
mesh_table.compute_vertex_normals()
mesh_table.translate(bbox_table.get_box_points()[0]) 
mesh_table.paint_uniform_color([0.6, 0.3, 0]) 

### Change bbox_box to mesh 
mesh_box = o3d.geometry.TriangleMesh.create_box(width = bound_box[0], height = bound_box[1], depth = bound_box[2]) 
mesh_box.compute_vertex_normals()
mesh_box.translate(bbox_box.get_box_points()[0]) 
mesh_box.paint_uniform_color([1, 1, 0])

msg = f"W: {bound_box[0]*1000:.0f}mm, L: {bound_box[1]*1000:.0f}mm, H: {bound_box[2]*1000:.0f}mm"
print(msg)

pcd_text = text_3d(msg, pos=bbox_box.get_box_points()[3], font_size=15)
pcd_text.paint_uniform_color([0, 0, 0])

# coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=bbox_table.get_box_points()[3])

o3d.visualization.draw_geometries([mesh_table, mesh_box, pcd_text],
                                  zoom = 0.50,
                                  front  = [ 0.23, -0.23, 0.95 ],
                                  lookat = [ 4.20e-05, -0.00, -1.54 ],
                                  up     = [ -0.95, 0.16, 0.27 ])
