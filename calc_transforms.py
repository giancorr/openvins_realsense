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

def invert_transform(tx, ty, tz, qx, qy, qz, qw):
    q_inv = [-qx, -qy, -qz, qw]
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

# We want to rotate the old FRD base_link by -90 deg yaw
# to align it with global_ned.
# Old FRD base_link: X=East, Y=South, Z=Down
# New FRD base_link: X=North, Y=East, Z=Down (Aligned with NED)

# Rotation to apply TO the old base_link to get the new base_link
# yaw = -90 deg = -math.pi/2
q_yaw_corr = euler_to_quaternion(-math.pi/2, 0, 0)

# 1. Front Camera (cam0)
tx_f, ty_f, tz_f = 0.12, 0.0, 0.14
q_f = [-0.5, -0.5, 0.5, 0.5] # Old base_link -> cam0_link
# We want New_base_link -> cam0_link.
# New_base -> cam = (New_base -> Old_base) * (Old_base -> cam)
# New_base -> Old_base is +90 deg yaw?
# Let's think: New X points North. Old X points East.
# In New frame (NED), East is +Y. So Old X is at +90 deg yaw relative to New X.
# So New_base -> Old_base is +90 deg yaw = +math.pi/2.
q_new_to_old = euler_to_quaternion(math.pi/2, 0, 0)

q_f_new = quaternion_multiply(q_new_to_old, q_f)
# Translation: old base -> cam is (0.12, 0, 0.14).
# Since old base is rotated by +90 deg (yaw) relative to new base:
# new_tx = old_tx * cos(90) - old_ty * sin(90) = -old_ty = 0
# new_ty = old_tx * sin(90) + old_ty * cos(90) = old_tx = 0.12
# new_tz = old_tz = 0.14
new_tx_f, new_ty_f, new_tz_f = 0.0, 0.12, 0.14

print("=== FRONT CAM (cam0) ===")
print(f"base_link -> cam: {new_tx_f} {new_ty_f} {new_tz_f} {q_f_new[0]:.7f} {q_f_new[1]:.7f} {q_f_new[2]:.7f} {q_f_new[3]:.7f}")
inv_tx_f, inv_ty_f, inv_tz_f, inv_q_f = invert_transform(new_tx_f, new_ty_f, new_tz_f, q_f_new[0], q_f_new[1], q_f_new[2], q_f_new[3])
print(f"imu -> base_link: {inv_tx_f:.6f} {inv_ty_f:.6f} {inv_tz_f:.6f} {inv_q_f[0]:.7f} {inv_q_f[1]:.7f} {inv_q_f[2]:.7f} {inv_q_f[3]:.7f}")

# 2. Back Camera (cam1)
tx_b, ty_b, tz_b = -0.15, 0.0, 0.20
q_b = [0.4304593, -0.4304593, 0.5609855, -0.5609855] # Old base_link -> cam1_link
q_b_new = quaternion_multiply(q_new_to_old, q_b)
new_tx_b = 0.0
new_ty_b = -0.15
new_tz_b = 0.20

print("\n=== BACK CAM (cam1) ===")
print(f"base_link -> cam: {new_tx_b} {new_ty_b} {new_tz_b} {q_b_new[0]:.7f} {q_b_new[1]:.7f} {q_b_new[2]:.7f} {q_b_new[3]:.7f}")
inv_tx_b, inv_ty_b, inv_tz_b, inv_q_b = invert_transform(new_tx_b, new_ty_b, new_tz_b, q_b_new[0], q_b_new[1], q_b_new[2], q_b_new[3])
print(f"imu -> base_link: {inv_tx_b:.6f} {inv_ty_b:.6f} {inv_tz_b:.6f} {inv_q_b[0]:.7f} {inv_q_b[1]:.7f} {inv_q_b[2]:.7f} {inv_q_b[3]:.7f}")
print(f"CPP Origin: tf2::Vector3({inv_tx_b:.6f}, {inv_ty_b:.6f}, {inv_tz_b:.6f})")
print(f"CPP Quat: tf2::Quaternion({inv_q_b[0]:.7f}, {inv_q_b[1]:.7f}, {inv_q_b[2]:.7f}, {inv_q_b[3]:.7f})")
