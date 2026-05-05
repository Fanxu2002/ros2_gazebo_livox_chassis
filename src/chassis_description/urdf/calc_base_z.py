import numpy as np
from stl import mesh
from scipy.spatial.transform import Rotation as R

# base_link 的 STL 路径
stl_path = "/home/shin/chassis/src/chassis_description/meshes/base_link.STL"

# 你现在已经确认正确的 rpy
roll = 3.14159265359
pitch = 1.57079632679
yaw = 3.14159265359

# 读取 STL
m = mesh.Mesh.from_file(stl_path)

# 所有顶点
points = m.vectors.reshape(-1, 3)

# 旋转矩阵，URDF 的 rpy 顺序就是 roll, pitch, yaw
rot = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()

# 旋转后的点
points_rot = points @ rot.T

# 找最低 z
min_z = points_rot[:, 2].min()
max_z = points_rot[:, 2].max()

# 要让最低点贴到 z=0，需要整体加 -min_z
z_offset = -min_z

print("base_link rotated min_z =", min_z)
print("base_link rotated max_z =", max_z)
print("recommended xyz z =", z_offset)
print()
print(f'<origin xyz="0 0 {z_offset:.9f}" rpy="{roll} {pitch} {yaw}" />')
