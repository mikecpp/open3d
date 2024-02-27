import open3d as o3d
import numpy as np

# chair = o3d.io.read_point_cloud("chair.pcd") 
# o3d.io.write_point_cloud("chair.pcd", chair, write_ascii=True) 
chair = o3d.io.read_point_cloud("20240226_15_10_38_PCL.ply")

bbox = chair.get_axis_aligned_bounding_box() 
bbox.color = (1, 0, 0) 

print("offset:", bbox.max_bound - bbox.min_bound) 

o3d.visualization.draw_geometries([chair, bbox]) 
