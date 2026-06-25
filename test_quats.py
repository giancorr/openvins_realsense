from scipy.spatial.transform import Rotation as R

q_front_old = R.from_quat([0.5047690, -0.4984747, 0.4937537, 0.5029300])
q_back_old = R.from_quat([0.3567783, -0.3462856, 0.6116574, 0.6153622])

# Try RotY(180)
print("q_front_old * RotY(180):", (q_front_old * R.from_euler('y', 180, degrees=True)).as_quat())
print("q_front_old * RotX(180):", (q_front_old * R.from_euler('x', 180, degrees=True)).as_quat())
print("q_front_old * RotZ(180):", (q_front_old * R.from_euler('z', 180, degrees=True)).as_quat())

# What is the relative rotation between q_front_old and q_back_old?
r_rel = q_front_old.inv() * q_back_old
print("Relative rotation front to back (Euler XYZ):", r_rel.as_euler('xyz', degrees=True))

