import numpy as np
from scipy.spatial.transform import Rotation as R

# Front camera
q_front = R.from_quat([0.5, 0.5, 0.5, 0.5]) # approx of user's front
r_front = q_front.as_matrix()

# Vector from base_link to cam0_link in base_link frame
T_base_cam = np.array([0.12, 0, 0.14])
# So vector from cam0_link to base_link in base_link frame
T_cam_base_in_base = -T_base_cam

# We need this vector in IMU frame.
# Assuming IMU center ~ cam0_link center.
T_imu_base_in_imu = r_front @ T_cam_base_in_base
print("Expected IMU->Base translation in IMU frame (Front):", T_imu_base_in_imu)

# Back camera
q_back = R.from_quat([-0.6123724, 0.6123724, 0.3535534, 0.3535534]) # This is base_link to cam1_link
r_back = q_back.as_matrix()

T_base_cam1 = np.array([-0.15, 0, 0.20])
T_cam1_base_in_base = -T_base_cam1

# But we want IMU->Base. The quaternion provided in back_session is IMU->Base!
# Let's check the user's q_back_old: [0.3567783, -0.3462856, 0.6116574, 0.6153622]
r_imu_base_back = R.from_quat([0.3567783, -0.3462856, 0.6116574, 0.6153622]).as_matrix()
T_imu_base_in_imu_back = r_imu_base_back @ T_cam1_base_in_base
print("Expected IMU->Base translation in IMU frame (Back):", T_imu_base_in_imu_back)

