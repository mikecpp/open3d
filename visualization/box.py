import open3d as o3d 
import open3d.visualization as vis 

cube = o3d.geometry.TriangleMesh.create_box(1, 2, 4)
vis.draw(cube)
