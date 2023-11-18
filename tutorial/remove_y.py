import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("./sample.ply")
o3d.visualization.draw([pcd])

points = np.asanyarray(pcd.points)

THRESHOLD = 0.0
# method 1
pcd.points = o3d.utility.Vector3dVector(points[points[:, 1] > THRESHOLD])

# method 2
# pcd = pcd.select_by_index(np.where(points[:, 1] > THRESHOLD)[0]) 

o3d.visualization.draw([pcd])
