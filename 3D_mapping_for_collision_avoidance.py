import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
import os
import time

folder_dir = "_out/"

def read_ply(path):
    cloud = o3d.io.read_point_cloud(path)
    return cloud

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

# initialize pointcloud instance.
pcd = (o3d.io.read_point_cloud("_out/00022637.ply"))

# include it in the visualizer before non-blocking visualization.
vis.add_geometry(pcd)

# to add new points each dt secs.
dt = 0.01
previous_t = time.time()

for pl in os.listdir(folder_dir):
    time.sleep(0.02)
    # check if the image ends with png
    if (pl.endswith(".ply")):

        if time.time() - previous_t > dt:
            # Reading new points
            path = folder_dir + pl
            cloud = read_ply(path)
            # o3d.visualization.draw_geometries([cloud])  # Visualize the point cloud

            # extending with ndarrays.
            pcd.points.extend(np.asarray(cloud.points))

            # Updating the map
            vis.update_geometry(pcd)
            previous_t = time.time()
        # Ploting the extended points
        keep_running = vis.poll_events()
        vis.update_renderer()
vis.destroy_window()