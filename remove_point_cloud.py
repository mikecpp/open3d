import open3d as o3d
import numpy as np
import time

dt = 3e-2 
t_total = 10 
n_new = 10 

vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

pcd = o3d.geometry.PointCloud()
points = np.random.rand(10, 3)
pcd.points = o3d.utility.Vector3dVector(points)
vis.add_geometry(pcd)

exec_times = []

current_t = time.time()
t0 = current_t
previous_t = current_t

while current_t - t0 < t_total:
    previous_t = time.time()
    while current_t - previous_t < dt:
        s = time.time()
        vis.remove_geometry(pcd)
        pcd = o3d.geometry.PointCloud()
        points = np.concatenate((points, np.random.rand(n_new, 3)))
        pcd.points = o3d.utility.Vector3dVector(points)
        vis.add_geometry(pcd)

        current_t = time.time()
        exec_times.append(time.time() - s)

    current_t = time.time()

    vis.poll_events()
    vis.update_renderer()

print(f"Without using extend\t# Points: {len(pcd.points)},\n"
      f"\t\t\t\t\t\tMean execution time:{np.mean(exec_times):.5f}")

vis.destroy_window()
