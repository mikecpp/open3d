import sys
import open3d as o3d
import numpy as np

dataset = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)
o3d.visualization.draw_geometries([pcd])

oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=75,
    outlier_ratio=0.75,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

print("Detected {} patches".format(len(oboxes)))

geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    # geometries.append(obox)
geometries.append(pcd)

o3d.visualization.draw_geometries(geometries)
