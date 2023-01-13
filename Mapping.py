import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
import os
import time

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

# initialize pointcloud instance.
pcd = o3d.geometry.PointCloud()
# *optionally* add initial points
points = np.random.rand(10, 3)
pcd.points = o3d.utility.Vector3dVector(points)

# include it in the visualizer before non-blocking visualization.
vis.add_geometry(pcd)

# to add new points each dt secs.
dt = 0.01
# number of points that will be added
n_new = 10

previous_t = time.time()

# run non-blocking visualization.
# To exit, press 'q' or click the 'x' of the window.
keep_running = True
while keep_running:

    if time.time() - previous_t > dt:
        # Options (uncomment each to try them out):
        # 1) extend with ndarrays.
        pcd.points.extend(np.random.rand(n_new, 3))

        # 2) extend with Vector3dVector instances.
        # pcd.points.extend(
        #     o3d.utility.Vector3dVector(np.random.rand(n_new, 3)))

        # 3) other iterables, e.g
        # pcd.points.extend(np.random.rand(n_new, 3).tolist())

        vis.update_geometry(pcd)
        previous_t = time.time()

    keep_running = vis.poll_events()
    vis.update_renderer()

vis.destroy_window()

# # folder_dir = "E:/CARLA_0.9.5/PythonAPI/MyProject/carla_dataset/_IMG/"
# folder_dir = "_out/"
# cloud = o3d.io.read_point_cloud("_out/00022638.ply")
# cloud2 = o3d.io.read_point_cloud("_out/00022639.ply")
# cloud3 = o3d.io.read_point_cloud("_out/00022640.ply")
#
# vis.add_geometry(cloud)
# o3d.visualization.draw_geometries([cloud])
# time.sleep(1)
# cloud.points.extend(np.asarray(cloud2.points))
# o3d.visualization.draw_geometries([cloud])

# co_ordinates = np.asarray(cloud.points)
# print(type(cloud))
# print(co_ordinates)
# DF = pd.DataFrame(co_ordinates)
# print(DF)
# # DF.to_csv("ply.csv")
# x = DF[0]
# y = DF[1]
# z = DF[2]
