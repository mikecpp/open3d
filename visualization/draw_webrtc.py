import open3d as o3d

if __name__ == "__main__":
    o3d.visualization.webrtc_server.enable_webrtc()
    cube_red = o3d.geometry.TriangleMesh.create_box(1, 2, 4)
    cube_red.compute_vertex_normals()
    cube_red.paint_uniform_color((0.0, 0.0, 1.0))
    o3d.visualization.draw(cube_red)
