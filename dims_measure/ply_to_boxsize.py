import open3d as o3d
import numpy as np

VOXEL_SIZE     = 0.001
NB_NEIGHBORS   = 40
STD_RATIO      = 2.0

TABLE_LEFT     = -0.30
TABLE_RIGHT    =  0.30

TABLE_DISTANCE = 0.01 
RANSAC_N       = 3
NUM_ITERS      = 1000

BOX_DISTANCE   = 0.015 

def ply_to_boxsize(filename): 
    pcd = o3d.io.read_point_cloud(filename) 

    ### Remove the non-Table objects 
    points = np.asarray(pcd.points) 
    points = points[points[:, 0] <=  TABLE_RIGHT] 
    points = points[points[:, 0] >=  TABLE_LEFT] 
    pcd.points = o3d.utility.Vector3dVector(points) 

    ### Down sample with voxel size 1mm 
    pcd = pcd.voxel_down_sample(voxel_size = VOXEL_SIZE) 
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors = NB_NEIGHBORS, std_ratio = STD_RATIO) 

    ### Get table top plane 
    plane_model, inliers = pcd.segment_plane(distance_threshold = TABLE_DISTANCE, ransac_n = RANSAC_N, num_iterations = NUM_ITERS)
    [a, b, c, d1] = plane_model
    pcd_table = pcd.select_by_index(inliers)
    other_pcd = pcd.select_by_index(inliers, invert=True)

    ### Get box top plane 
    plane_model, inliers = other_pcd.segment_plane(distance_threshold = BOX_DISTANCE, ransac_n = RANSAC_N, num_iterations = NUM_ITERS)
    [a, b, c, d2] = plane_model

    ### Calculate box height from 2 planes
    depth = d1 - d2 
    pcd_box = other_pcd.select_by_index(inliers)

    ### Pick up any box point plus depth 
    points = np.asarray(pcd_box.points) 
    points[-100, 2] -= (depth - .005)
    pcd_box.points = o3d.utility.Vector3dVector(points)

    ### Get box bounding box as box length, width & height 
    bbox_box = pcd_box.get_axis_aligned_bounding_box() 
    bound_box = bbox_box.get_max_bound() - bbox_box.get_min_bound()

    width  = bound_box[0] * 1000 
    length = bound_box[1] * 1000 
    height = bound_box[2] * 1000 

    return (width, length, height, bbox_box)
