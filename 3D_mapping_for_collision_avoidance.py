import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
import os
import time

folder_dir = "_test_lidar/"

def read_ply(path):
    cloud = o3d.io.read_point_cloud(path)
    return cloud

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

# initialize pointcloud instance.
pcd = (o3d.io.read_point_cloud("_lidar/010670.ply"))

# include it in the visualizer before non-blocking visualization.
vis.add_geometry(pcd)

# to add new points each dt secs.
dt = 0.01
previous_t = time.time()

for pl in os.listdir(folder_dir):
    time.sleep(0.1)
    # check if the image ends with png
    if (pl.endswith(".ply")):

        if time.time() - previous_t > dt:
            # Reading new points
            path = folder_dir + pl
            cloud = read_ply(path)
            # o3d.visualization.draw_geometries([cloud])  # Visualize the point cloud

            # extending with ndarrays.
            # pcd.points.extend(np.asarray(cloud.points))

            # Updating the points
            pcd.points = cloud.points

            # Updating the map
            vis.update_geometry(pcd)
            # vis.update_geometry(cloud)

        # Ploting the extended points

        vis.update_renderer()
        vis.poll_events()
vis.destroy_window()