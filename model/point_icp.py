import sys
import open3d as o3d
import numpy as np
import copy

paths = [
    "20231123_13_29_05_PCL,ply", "20231123_13_29_16_PCL.ply", 
    "20231123_13_29_26_PCL.ply", "20231123_13_29_35_PCL.ply", "20231123_13_29_46_PCL.ply",    
    "20231123_13_27_00_PCL.ply", "20231123_13_27_18_PCL.ply", "20231123_13_27_32_PCL.ply", 
    "20231123_13_27_46_PCL.ply", "20231123_13_28_04_PCL.ply", "20231123_13_28_41_PCL.ply",
]

def load_point_clouds(voxel_size=0.0):
    pcds = []
    for path in paths:
        pcd = o3d.io.read_point_cloud(path)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source, target):
    source.estimate_normals()
    target.estimate_normals()
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)

    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

voxel_size = 0.01
pcds_down = load_point_clouds(voxel_size)

max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine   = voxel_size * 1.5
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    pose_graph = full_registration(pcds_down, 
                                   max_correspondence_distance_coarse, 
                                   max_correspondence_distance_fine)

pcd_combined = o3d.geometry.PointCloud()
for point_id in range(len(pcds_down)):
    print(pose_graph.nodes[point_id].pose)
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    pcd_combined += pcds_down[point_id]

o3d.io.write_point_cloud("point_final.ply", pcd_combined, write_ascii=True)

# o3d.visualization.draw_geometries([pcd_combined])
pcd, ind = pcd_combined.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
o3d.visualization.draw([pcd])
