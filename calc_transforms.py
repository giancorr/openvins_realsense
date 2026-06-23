import math

def invert_transform(tx, ty, tz, qx, qy, qz, qw):
    # R_inv = q_conjugate
    q_inv = [-qx, -qy, -qz, qw]
    
    # t_inv = -R_inv * t
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

# Front camera (cam0) in base_link (FLU)
# User said: 12cm forward, 14cm down.
tx, ty, tz = 0.12, 0.0, -0.14
qx, qy, qz, qw = 0.5, 0.5, 0.5, 0.5 # FLU to IMU

print("--- BASE_LINK -> CAM0_LINK ---")
print(f"TF: {tx} {ty} {tz} {qx} {qy} {qz} {qw} base_link cam0_link")

inv_tx, inv_ty, inv_tz, inv_q = invert_transform(tx, ty, tz, qx, qy, qz, qw)
print("\n--- IMU (CAM0) -> BASE_LINK ---")
print(f"TF: {inv_tx} {inv_ty} {inv_tz} {inv_q[0]} {inv_q[1]} {inv_q[2]} {inv_q[3]} imu base_link")

# Back camera (cam1) in base_link (FLU)
# User said: 14cm back, 18cm down, 15 deg pitch down.
import math
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

# Back camera orientation
# Base rotation * yaw 180 * pitch 15
q_cam_standard = [0.5, 0.5, 0.5, 0.5]
yaw_rot = math.pi
pitch_rot = 15 * math.pi / 180
q_mount = euler_to_quaternion(yaw_rot, pitch_rot, 0)
q_back_base = quaternion_multiply(q_mount, q_cam_standard)

tx2, ty2, tz2 = -0.14, 0.0, -0.18
print("\n--- BASE_LINK -> CAM1_LINK ---")
print(f"TF: {tx2} {ty2} {tz2} {q_back_base[0]} {q_back_base[1]} {q_back_base[2]} {q_back_base[3]} base_link cam1_link")

inv_tx2, inv_ty2, inv_tz2, inv_q2 = invert_transform(tx2, ty2, tz2, q_back_base[0], q_back_base[1], q_back_base[2], q_back_base[3])
print("\n--- CAM1_IMU -> BASE_LINK (For CPP) ---")
print(f"Origin: {inv_tx2}, {inv_ty2}, {inv_tz2}")
print(f"Quat: {inv_q2[0]}, {inv_q2[1]}, {inv_q2[2]}, {inv_q2[3]}")
