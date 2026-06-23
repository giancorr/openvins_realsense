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

# The correct mapping from FRD to T265 IMU (X Fwd, Y Left, Z Up)
# This is a 180 degree rotation around X!
q_frd_to_imu = [1.0, 0.0, 0.0, 0.0]

print("=== FRONT CAM (cam0) ===")
tx_f, ty_f, tz_f = 0.12, 0.0, 0.14
q_f = q_frd_to_imu
print(f"base_link -> cam: {tx_f} {ty_f} {tz_f} {q_f[0]:.7f} {q_f[1]:.7f} {q_f[2]:.7f} {q_f[3]:.7f}")
inv_tx_f, inv_ty_f, inv_tz_f, inv_q_f = invert_transform(tx_f, ty_f, tz_f, q_f[0], q_f[1], q_f[2], q_f[3])
print(f"imu -> base_link: {inv_tx_f:.6f} {inv_ty_f:.6f} {inv_tz_f:.6f} {inv_q_f[0]:.7f} {inv_q_f[1]:.7f} {inv_q_f[2]:.7f} {inv_q_f[3]:.7f}")

print("\n=== BACK CAM (cam1) - 30 deg pitch ===")
tx, ty, tz = -0.15, 0.0, 0.20
q_mount = euler_to_quaternion(math.pi, 30 * math.pi / 180, 0)
q_back = quaternion_multiply(q_mount, q_frd_to_imu)
print(f"base_link -> cam: {tx} {ty} {tz} {q_back[0]:.7f} {q_back[1]:.7f} {q_back[2]:.7f} {q_back[3]:.7f}")
inv_tx, inv_ty, inv_tz, inv_q = invert_transform(tx, ty, tz, q_back[0], q_back[1], q_back[2], q_back[3])
print(f"imu -> base_link: {inv_tx:.6f} {inv_ty:.6f} {inv_tz:.6f} {inv_q[0]:.7f} {inv_q[1]:.7f} {inv_q[2]:.7f} {inv_q[3]:.7f}")
