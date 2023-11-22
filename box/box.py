import numpy as np
import open3d as o3d

mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0) 
o3d.visualization.draw([mesh_box]) 
