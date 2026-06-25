import numpy as np
from scipy.spatial.transform import Rotation as R

# ENU to ESD
R_globalned = R.from_quat([1, 0, 0, 0]).as_matrix() # RotX(180)

# Front Camera
# IMU in ENU
R_imu_global_front = np.array([[1,0,0], [0,0,1], [0,-1,0]])
# Derived base_link in IMU
R_base_imu_front = np.array([[0,1,0], [0,0,1], [1,0,0]])

R_base_globalned_front = R_globalned @ R_imu_global_front @ R_base_imu_front
print("Front base_link in globalned:")
print(R_base_globalned_front)
print("Front quaternion:", R.from_matrix(R_base_imu_front).as_quat())

# Back Camera
# IMU in WSU (global X = West, Z = Up -> Y = South)
# IMU X = West, Y = Down, Z = South
# X_imu = X_global, Y_imu = -Z_global, Z_imu = Y_global
R_imu_global_back = np.array([[1,0,0], [0,0,1], [0,-1,0]])
# Derived base_link in IMU
R_base_imu_back = np.array([[0,-1,0], [0,0,1], [-1,0,0]])

R_base_globalned_back = R_globalned @ R_imu_global_back @ R_base_imu_back
print("\nBack base_link in globalned:")
print(R_base_globalned_back)
print("Back quaternion:", R.from_matrix(R_base_imu_back).as_quat())
