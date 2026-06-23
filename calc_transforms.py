import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. We define base_link -> cam0_link such that X_base(Fwd) = Z_cam0, Y_base(Right) = X_cam0, Z_base(Down) = Y_cam0
# This maps FRD to OpenCV.
# Matrix from base_link to cam0:
# [P_cam0] = R_cam0_base * [P_base]
R_cam0_base = np.array([
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 0]
])
# Translation base_link -> cam0 is [0.12, 0.0, 0.14] in base_link frame.
# So T_cam0_base has translation -R_cam0_base * t_base_cam0 = - [0, 0.14, 0.12]
t_base_cam0 = np.array([0.12, 0.0, 0.14])

# Let's get the quaternion for base_link -> cam0_link
q_cam0_base = R.from_matrix(R_cam0_base).as_quat()
print(f"base_link -> cam0_link (FRD to OpenCV):")
print(f"Translation: {t_base_cam0[0]:.6f} {t_base_cam0[1]:.6f} {t_base_cam0[2]:.6f}")
print(f"Quaternion: {q_cam0_base[0]:.7f} {q_cam0_base[1]:.7f} {q_cam0_base[2]:.7f} {q_cam0_base[3]:.7f}")

# 2. Kalibr gives us T_cam0_imu, which maps points from imu to cam0
# [P_cam0] = R_cam0_imu * [P_imu] + t_cam0_imu
T_cam0_imu = np.array([
    [-0.9998761771972654, 0.002831174545748783, 0.015479493663382282, 0.009847737933630705],
    [-0.0029326606695474597, -0.999974330610803, -0.0065374016913607265, 0.004044777973851876],
    [0.015460587788970944, -0.006581988314211809, 0.9998588138607628, -0.015241042987496874],
    [0.0, 0.0, 0.0, 1.0]
])

# We need imu -> base_link for TF publisher.
# This means we need T_base_imu.
# T_cam0_base * T_base_imu = T_cam0_imu
# Therefore: T_base_imu = (T_cam0_base)^-1 * T_cam0_imu = T_base_cam0 * T_cam0_imu

# Construct T_base_cam0
T_base_cam0 = np.eye(4)
T_base_cam0[:3, :3] = R_cam0_base.T  # Inverse of rotation
T_base_cam0[:3, 3] = t_base_cam0

T_base_imu = T_base_cam0 @ T_cam0_imu

# Now we invert T_base_imu to get imu -> base_link (T_imu_base) for TF tree
T_imu_base = np.linalg.inv(T_base_imu)
t_imu_base = T_imu_base[:3, 3]
q_imu_base = R.from_matrix(T_imu_base[:3, :3]).as_quat()

print(f"\nimu -> base_link (Front Camera):")
print(f"Translation: {t_imu_base[0]:.6f} {t_imu_base[1]:.6f} {t_imu_base[2]:.6f}")
print(f"Quaternion: {q_imu_base[0]:.7f} {q_imu_base[1]:.7f} {q_imu_base[2]:.7f} {q_imu_base[3]:.7f}")

# 3. For the BACK camera
# T_base_cam1: mounted facing backward, 30 deg pitch down.
t_base_cam1 = np.array([-0.15, 0.0, 0.20])
# Start with cam0 orientation relative to base (FRD to OpenCV)
# Then rotate it 180 yaw, 30 pitch down.
R_mount = R.from_euler('ZYX', [180, 30, 0], degrees=True).as_matrix()
R_base_cam1 = R_mount @ R_cam0_base.T
T_base_cam1 = np.eye(4)
T_base_cam1[:3, :3] = R_base_cam1
T_base_cam1[:3, 3] = t_base_cam1

q_cam1_base = R.from_matrix(R_base_cam1.T).as_quat()
print(f"\nbase_link -> cam1_link (Back Camera):")
print(f"Translation: {t_base_cam1[0]:.6f} {t_base_cam1[1]:.6f} {t_base_cam1[2]:.6f}")
print(f"Quaternion: {q_cam1_base[0]:.7f} {q_cam1_base[1]:.7f} {q_cam1_base[2]:.7f} {q_cam1_base[3]:.7f}")

T_cam1_imu = np.array([
    [0.9998761771972654, 0.002831174545748783, -0.015479493663382282, 0.006442249327686602],
    [0.0029326606695474597, -0.999974330610803, 0.0065374016913607265, 0.005483006345951236],
    [-0.015460587788970944, -0.006581988314211809, -0.9998588138607628, -0.2352099820368647],
    [0.0, 0.0, 0.0, 1.0]
])

T_base_imu1 = T_base_cam1 @ T_cam1_imu
T_imu1_base = np.linalg.inv(T_base_imu1)
t_imu1_base = T_imu1_base[:3, 3]
q_imu1_base = R.from_matrix(T_imu1_base[:3, :3]).as_quat()

print(f"\nimu -> base_link (Back Camera):")
print(f"Translation: {t_imu1_base[0]:.6f} {t_imu1_base[1]:.6f} {t_imu1_base[2]:.6f}")
print(f"Quaternion: {q_imu1_base[0]:.7f} {q_imu1_base[1]:.7f} {q_imu1_base[2]:.7f} {q_imu1_base[3]:.7f}")

