import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
import os

# folder_dir = "E:/CARLA_0.9.5/PythonAPI/MyProject/carla_dataset/_IMG/"
folder_dir = "_out/"

cloud = o3d.io.read_point_cloud("_out/00022638.ply")
# o3d.visualization.draw_geometries([cloud])
co_ordinates = np.asarray(cloud.points)
print(type(cloud))
print(co_ordinates)
DF = pd.DataFrame(co_ordinates)
print(DF)
DF.to_csv("ply.csv")