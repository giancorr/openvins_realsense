from scipy.spatial.transform import Rotation as R

# User's q_front: x, y, z, w
q_front_old = [0.5047690, -0.4984747, 0.4937537, 0.5029300]
r_front_old = R.from_quat(q_front_old)

# Rotate by -90 around local Z
r_corr = R.from_euler('z', -90, degrees=True)
r_front_new = r_front_old * r_corr
print("New q_front (RotZ -90):", r_front_new.as_quat())

r_corr_plus = R.from_euler('z', 90, degrees=True)
r_front_new_plus = r_front_old * r_corr_plus
print("New q_front (RotZ +90):", r_front_new_plus.as_quat())

# User's OLD q_back that "worked well":
# 0.3567783, -0.3462856, 0.6116574, 0.6153622
q_back_old_worked = [0.3567783, -0.3462856, 0.6116574, 0.6153622]
print("Old q_back that worked:", q_back_old_worked)

# What if we apply RotZ(180) to user's OLD q_back?
r_back_old = R.from_quat(q_back_old_worked)
r_back_180 = r_back_old * R.from_euler('z', 180, degrees=True)
print("Old q_back * RotZ(180):", r_back_180.as_quat())

