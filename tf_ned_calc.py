import numpy as np
from scipy.spatial.transform import Rotation as R

# ============================================================
# PHYSICAL SETUP (NED base_link, camera optical convention)
# ============================================================
# base_link NED: X=forward, Y=right, Z=down
# cam0: 11cm davanti, guarda avanti (Z esce dalla lente = forward)
# cam1: 11cm dietro, guarda indietro (Z esce dalla lente = backward)
# Camera optical: X=right, Y=down, Z=forward (out of lens)

# ============================================================
# 1. global -> global_ned
# ============================================================
# global (OpenVINS, ENU-like): X~East, Y~North, Z=Up
# global_ned (NED):             X=North, Y=East,  Z=Down
# Axes of global_ned expressed in global:
#   X_ned = Y_global (North)  = [0, 1, 0]
#   Y_ned = X_global (East)   = [1, 0, 0]
#   Z_ned = -Z_global (Down)  = [0, 0, -1]
R_global_globalned = R.from_matrix([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])
q = R_global_globalned.as_quat()  # [qx, qy, qz, qw]
print("=" * 60)
print("global -> global_ned (180° around X)")
print(f"  Rotation matrix:\n{R_global_globalned.as_matrix()}")
print(f"  Quaternion: qx={q[0]:.6f} qy={q[1]:.6f} qz={q[2]:.6f} qw={q[3]:.6f}")
print(f"  CMD: ros2 run tf2_ros static_transform_publisher 0 0 0 {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} global global_ned")

# ============================================================
# 2. imu -> base_link (NED)
# ============================================================
# Il vecchio transform FLU funzionava visivamente:
#   0 0 -0.11 yaw=-pi/2 pitch=-pi/2 roll=0
# R_imu_base_flu = R.from_euler('ZYX', [-pi/2, -pi/2, 0])
# Per passare da FLU a NED: R_flu_ned flips Y and Z
# R_imu_base_ned = R_imu_base_flu * R_flu_ned

R_imu_base_flu = R.from_euler('ZYX', [-np.pi/2, -np.pi/2, 0])
R_flu_ned = R.from_matrix([
    [1,  0,  0],
    [0, -1,  0],
    [0,  0, -1]
])
R_imu_base_ned = R_imu_base_flu * R_flu_ned

q = R_imu_base_ned.as_quat()
print("\n" + "=" * 60)
print("imu -> base_link (NED)")
print(f"  Rotation matrix:\n{R_imu_base_ned.as_matrix().round(6)}")
print(f"  Quaternion: qx={q[0]:.6f} qy={q[1]:.6f} qz={q[2]:.6f} qw={q[3]:.6f}")
print(f"  CMD: ros2 run tf2_ros static_transform_publisher 0 0 -0.11 {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} imu base_link")

# ============================================================
# 3. base_link (NED) -> cam0 (looking forward)
# ============================================================
# cam0 optical axes expressed in base_link NED coordinates:
#   cam0_X (right)   = base_Y (right)   = [0, 1, 0]
#   cam0_Y (down)    = base_Z (down)    = [0, 0, 1]
#   cam0_Z (forward) = base_X (forward) = [1, 0, 0]
# R_base_cam0 columns = [cam0_X_in_base, cam0_Y_in_base, cam0_Z_in_base]
R_base_cam0 = R.from_matrix([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]
])
q = R_base_cam0.as_quat()
print("\n" + "=" * 60)
print("base_link -> cam0 (Z=forward, out of lens)")
print(f"  Rotation matrix:\n{R_base_cam0.as_matrix()}")
print(f"  Quaternion: qx={q[0]:.6f} qy={q[1]:.6f} qz={q[2]:.6f} qw={q[3]:.6f}")
print(f"  CMD: ros2 run tf2_ros static_transform_publisher 0.11 0 0 {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} base_link cam0_link")

# ============================================================
# 4. base_link (NED) -> cam1 (looking backward)
# ============================================================
# cam1 optical axes expressed in base_link NED coordinates:
#   cam1_X (right from cam1 POV) = -base_Y (left) = [0, -1, 0]
#   cam1_Y (down)                = base_Z (down)  = [0,  0, 1]
#   cam1_Z (backward = out of lens) = -base_X     = [-1, 0, 0]
R_base_cam1 = R.from_matrix([
    [ 0, 0, -1],
    [-1, 0,  0],
    [ 0, 1,  0]
])
q = R_base_cam1.as_quat()
print("\n" + "=" * 60)
print("base_link -> cam1 (Z=backward, out of lens)")
print(f"  Rotation matrix:\n{R_base_cam1.as_matrix()}")
print(f"  Quaternion: qx={q[0]:.6f} qy={q[1]:.6f} qz={q[2]:.6f} qw={q[3]:.6f}")
print(f"  CMD: ros2 run tf2_ros static_transform_publisher -0.11 0 0 {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} base_link cam1_link")

# ============================================================
# 5. imu_back -> base_link (NED) — for cam1's IMU
# ============================================================
# cam1 is mounted facing backward (180° yaw around body Y axis)
# cam1's IMU axes in body frame:
#   imu1_X = body right (was body left for cam0)
#   imu1_Y = body up (same as cam0)
#   imu1_Z = body backward (was body forward for cam0)
#
# R_imu1_base = Ry(pi) * R_imu0_base
R_y_pi = R.from_matrix([
    [-1, 0,  0],
    [ 0, 1,  0],
    [ 0, 0, -1]
])
R_imu1_base_ned = R_y_pi * R_imu_base_ned

q = R_imu1_base_ned.as_quat()
print("\n" + "=" * 60)
print("imu_back -> base_link (NED) — cam1's IMU")
print(f"  Rotation matrix:\n{R_imu1_base_ned.as_matrix().round(6)}")
print(f"  Quaternion: qx={q[0]:.6f} qy={q[1]:.6f} qz={q[2]:.6f} qw={q[3]:.6f}")
print(f"  CMD: ros2 run tf2_ros static_transform_publisher 0 0 -0.11 {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} imu_back base_link")
# Translation is [0, 0, -0.11] in imu_back frame:
# cam1's IMU Z = body backward, base_link is 11cm in front (body forward = -Z)
# so tz = -0.11

# ============================================================
# Verification: check all rotation matrices are proper (det=+1)
# ============================================================
print("\n" + "=" * 60)
print("VERIFICATION:")
for name, rot in [("global_ned", R_global_globalned), ("imu->base", R_imu_base_ned), 
                   ("imu_back->base", R_imu1_base_ned),
                   ("base->cam0", R_base_cam0), ("base->cam1", R_base_cam1)]:
    det = np.linalg.det(rot.as_matrix())
    print(f"  {name}: det={det:.4f} (should be 1.0)")
