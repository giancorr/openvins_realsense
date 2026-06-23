import math
import numpy as np

def euler_to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def invert_transform(tx, ty, tz, qx, qy, qz, qw):
    # R_inv = q_conjugate
    q_inv = [-qx, -qy, -qz, qw]
    
    # t_inv = -R_inv * t
    # Convert q_inv to matrix
    w, x, y, z = qw, -qx, -qy, -qz
    R11 = 1 - 2*y**2 - 2*z**2
    R12 = 2*x*y - 2*z*w
    R13 = 2*x*z + 2*y*w
    R21 = 2*x*y + 2*z*w
    R22 = 1 - 2*x**2 - 2*z**2
    R23 = 2*y*z - 2*x*w
    R31 = 2*x*z - 2*y*w
    R32 = 2*y*z + 2*x*w
    R33 = 1 - 2*x**2 - 2*y**2
    
    rx = R11*(-tx) + R12*(-ty) + R13*(-tz)
    ry = R21*(-tx) + R22*(-ty) + R23*(-tz)
    rz = R31*(-tx) + R32*(-ty) + R33*(-tz)
    
    return rx, ry, rz, q_inv

# The base to camera rotation is a combination of:
# 1) Base_link to Camera Link default orientation for RealSense T265.
# Standard T265 base orientation (when facing forward) is pitch -90 degrees? No, the T265 lens faces forward, but the ROS TF convention for cameras is Z forward, X right, Y down.
# Let's check the old q_front: [-0.5, -0.5, -0.5, 0.5] which is base->cam = [0.5, 0.5, 0.5, 0.5] (yaw=-90, pitch=0, roll=-90)
q_cam_standard = [0.5, 0.5, 0.5, 0.5]

print("--- FRONT CAM ---")
# Front cam is X=0, Y=0, Z=-0.14
# The orientation is the standard forward-facing orientation.
t_front_base = [0, 0, -0.14]
inv_tx, inv_ty, inv_tz, inv_q = invert_transform(0, 0, -0.14, 0.5, 0.5, 0.5, 0.5)
print(f"tf2 publisher: 0 0 -0.14 0.5 0.5 0.5 0.5")
print(f"C++ Origin: tf2::Vector3({inv_tx}, {inv_ty}, {inv_tz})")
print(f"C++ Quat: tf2::Quaternion({inv_q[0]}, {inv_q[1]}, {inv_q[2]}, {inv_q[3]})")


print("\n--- BACK CAM ---")
# Back cam is X=-0.14, Y=0, Z=-0.18
# Orientation: facing backward (Yaw = 180 degrees) and pitched 15 degrees down
# Pitching "down" means rotating around its own X axis? No, in drone FLU frame, pitch down is positive pitch (rotation around Y axis).
# Wait, let's just create the rotation from base to the camera body:
# Rotate yaw by 180 (math.pi). Pitch by 15 deg (15 * math.pi / 180).
yaw_rot = math.pi
pitch_rot = 15 * math.pi / 180 # Pitch down is positive in FLU
q_mount = euler_to_quaternion(yaw_rot, pitch_rot, 0)

# Multiply q_mount by q_cam_standard to get the final orientation
q_back_base = quaternion_multiply(q_mount, q_cam_standard)

inv_tx_b, inv_ty_b, inv_tz_b, inv_q_b = invert_transform(-0.14, 0, -0.18, q_back_base[0], q_back_base[1], q_back_base[2], q_back_base[3])

print(f"tf2 publisher: -0.14 0 -0.18 {q_back_base[0]} {q_back_base[1]} {q_back_base[2]} {q_back_base[3]}")
print(f"C++ Origin: tf2::Vector3({inv_tx_b}, {inv_ty_b}, {inv_tz_b})")
print(f"C++ Quat: tf2::Quaternion({inv_q_b[0]}, {inv_q_b[1]}, {inv_q_b[2]}, {inv_q_b[3]})")
