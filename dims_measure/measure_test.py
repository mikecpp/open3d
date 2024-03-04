import sys
import time
import open3d as o3d
import numpy as np
from ply_to_boxsize import ply_to_boxsize

BOX_WIDTH  = 185
BOX_LENGTH = 300
BOX_DEPTH  = 210

def measure(filename, draw = False):
    (width, length, depth, bbox) = ply_to_boxsize(filename)
    print("Filename: ", filename)
    print(f"Width:  {width:.0f}mm  ({BOX_WIDTH - width:.0f}mm)")
    print(f"Length: {length:.0f}mm  ({BOX_LENGTH - length:.0f}mm)")
    print(f"Depth:  {depth:.0f}mm  ({BOX_DEPTH - depth:.0f}mm)")
    if draw:
        pcd = o3d.io.read_point_cloud(filename) 
        bbox.color = (1, 0, 0)
        o3d.visualization.draw_geometries([pcd, bbox]) 

measure(sys.argv[1], True)

'''
for i in range(10):
    time_start = time.time() 
    measure(sys.argv[1])
    time_end = time.time()
    print(f"time elapse: {(time_end - time_start)*1000:.0f} ms")
    print("-----------------------------------------------")
'''