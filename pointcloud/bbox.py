import open3d as o3d
import numpy as np

demo_crop_data = o3d.data.DemoCropPointCloud()
pcd = o3d.io.read_point_cloud(demo_crop_data.point_cloud_path)
vol = o3d.visualization.read_selection_polygon_volume(demo_crop_data.cropped_json_path)
chair = vol.crop_point_cloud(pcd)

aabb = pcd.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)

obb = pcd.get_oriented_bounding_box()
obb.color  = (0, 1, 0)

o3d.visualization.draw_geometries([pcd, aabb, obb])
