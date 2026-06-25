import numpy as np
from scipy.spatial.transform import Rotation as R

# base_link to cam1_link
q_base_cam1 = [-0.6123724, 0.6123724, 0.3535534, 0.3535534] # x, y, z, w
r_base_cam1 = R.from_quat(q_base_cam1)
r_cam1_base = r_base_cam1.inv()
q_cam1_base = r_cam1_base.as_quat()
print("q_cam1_base:", q_cam1_base)

# Translation: base_link to cam1_link in base_link
T_base_cam1 = np.array([-0.15, 0, 0.20])
T_cam1_base_in_base = -T_base_cam1

# T_cam1_base in cam1_link frame (which is IMU frame)
T_cam1_base_in_cam1 = r_cam1_base.apply(T_cam1_base_in_base)
print("T_cam1_base_in_cam1:", T_cam1_base_in_cam1)
