import numpy as np
import open3d as o3d
import os

# folder_dir = "E:/CARLA_0.9.5/PythonAPI/MyProject/carla_dataset/_IMG/"
folder_dir = "_out/"

def read_ply(path):
    cloud = o3d.io.read_point_cloud(path)
    # cloud = read_point_cloud("_out/00022637.ply") # Read the point cloud
    o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud


for pl in os.listdir(folder_dir):

    # check if the image ends with png
    if (pl.endswith(".ply")):
        path = folder_dir + pl
        read_ply(path)



# import open3d as o3d;
# print(o3d.__version__)
# mesh = o3d.geometry.TriangleMesh.create_sphere();
# mesh.compute_vertex_normals();
# o3d.visualization.draw(mesh, raw_mode=True)