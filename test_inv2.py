import numpy as np
from scipy.spatial.transform import Rotation as R

q_cam1 = [-0.6123724, 0.6123724, 0.3535534, 0.3535534]
t_cam1 = np.array([-0.15, 0, 0.20])

r1 = R.from_quat(q_cam1)
r1_inv = r1.inv()
t1_inv = r1_inv.apply(-t_cam1)

print("cam1 inverse q (x y z w):", r1_inv.as_quat())
print("cam1 inverse t (x y z):", t1_inv)

