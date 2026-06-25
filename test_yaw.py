import numpy as np
from scipy.spatial.transform import Rotation as R

# Supponiamo che global_base abbia un certo roll, pitch, yaw
r = R.from_euler('xyz', [180, 0, 45], degrees=True)

# Come lo estrae il C++ tf2::Matrix3x3::getRPY?
# getRPY in tf2 returns roll, pitch, yaw in fixed axis X, Y, Z (extrinsic) or intrinsic?
# tf2 getRPY is equivalent to euler from matrix with 'xyz' extrinsic.
roll, pitch, yaw = r.as_euler('xyz')
print("Extracted RPY:", roll, pitch, yaw)

# C++ fa:
# q_yaw.setRPY(0.0, 0.0, yaw)
r_yaw = R.from_euler('xyz', [0, 0, yaw])

r_odom_base = r_yaw.inv() * r
print("odom_base RPY:", r_odom_base.as_euler('xyz', degrees=True))

