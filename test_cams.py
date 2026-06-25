from scipy.spatial.transform import Rotation as R

q_cam0 = R.from_quat([0.5, 0.5, 0.5, 0.5])
q_cam1 = R.from_quat([-0.6123724, 0.6123724, 0.3535534, 0.3535534])

r_rel = q_cam0.inv() * q_cam1
print("Relative rotation cam0 to cam1 (Euler XYZ):", r_rel.as_euler('xyz', degrees=True))

