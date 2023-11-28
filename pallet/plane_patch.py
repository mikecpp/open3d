import sys
import open3d as o3d
import numpy as np

THRESHOLD  = -0.45
VOXEL_SIZE =  0.02

pcd = o3d.io.read_point_cloud("pallet_01.ply") 
pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
points = np.asarray(pcd.points) 
pcd.points = o3d.utility.Vector3dVector(points[points[:, 1] >= THRESHOLD])
# o3d.visualization.draw([pcd])

pcd.estimate_normals()

oboxes = pcd.detect_planar_patches()
print("Detected {} patches".format(len(oboxes)))

geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    # geometries.append(obox)
pcd.paint_uniform_color([0.0, 0.0, 0.0])
geometries.append(pcd)

o3d.visualization.draw_geometries(geometries)
