import open3d as o3d
import numpy as np
import time

vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

pcd = o3d.geometry.PointCloud()
points = np.random.rand(10, 3)
pcd.points = o3d.utility.Vector3dVector(points)
vis.add_geometry(pcd)

dt = 0.01
n_new = 10

previous_t = time.time()

keep_running = True
while keep_running:
    
    if time.time() - previous_t > dt:
        pcd.points.extend(np.random.rand(n_new, 3))       
        vis.update_geometry(pcd)
        previous_t = time.time()

    keep_running = vis.poll_events()
    vis.update_renderer()

vis.destroy_window()
