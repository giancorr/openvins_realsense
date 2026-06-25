import numpy as np
from scipy.spatial.transform import Rotation as R

q2 = [0.35355339, -0.35355339, -0.61237244, 0.61237244] # x, y, z, w
r2 = R.from_quat(q2)
r2_inv = r2.inv()
print("q2 inverse:", r2_inv.as_quat())

T_base_cam = np.array([-0.15, 0, 0.20])
T_cam_base_in_base = -T_base_cam
T_cam_base_in_cam = r2_inv.apply(T_cam_base_in_base)
print("T_cam_base_in_cam:", T_cam_base_in_cam)

