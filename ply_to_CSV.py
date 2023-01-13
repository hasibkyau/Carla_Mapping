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
# DF.to_csv("ply.csv")
x = DF[0]
y = DF[1]
z = DF[2]


ax = plt.figure().add_subplot(projection='3d')
ax.plot(x, y, zs=0, zdir='z', label='curve in (x, y)')
ax.scatter(x, y, zs=0, zdir='y', label='points in (x, z)')

# Make legend, set axes limits and labels
ax.legend()
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Customize the view angle so it's easier to see that the scatter points lie
# on the plane y=0
ax.view_init(elev=20., azim=-35)

plt.show()