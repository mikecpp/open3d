import open3d as o3d
import open3d.core as o3c
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.t.io.read_point_cloud(ply_point_cloud.path)
downpcd = pcd.voxel_down_sample(voxel_size=0.03)
downpcd.estimate_normals(max_nn=30, radius=0.1)
# o3d.visualization.draw_geometries([downpcd.to_legacy()], point_show_normal=True)

normals = downpcd.point.normals
print("Print first 5 normals of the downsampled point cloud.")
print(normals[500:520], "\n")

print("Convert normals tensor into numpy array.")
normals_np = normals.numpy()
print(normals_np[500:520])
