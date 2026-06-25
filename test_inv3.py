import numpy as np
from scipy.spatial.transform import Rotation as R

q_cam0 = [0.35355339, -0.35355339, -0.61237244, 0.61237244]
t_cam0 = np.array([-0.15, 0, 0.20])

r = R.from_quat(q_cam0)
r_inv = r.inv()
t_inv = r_inv.apply(-t_cam0)

print("cam0 inverse q (x y z w):", r_inv.as_quat())
print("cam0 inverse t (x y z):", t_inv)

