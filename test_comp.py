from scipy.spatial.transform import Rotation as R

q1 = [-0.6123724, 0.6123724, 0.3535534, 0.3535534]
q2 = [0.35355339, -0.35355339, -0.61237244, 0.61237244]

r1 = R.from_quat(q1)
r2 = R.from_quat(q2)

print("r1 Euler:", r1.as_euler('xyz', degrees=True))
print("r2 Euler:", r2.as_euler('xyz', degrees=True))
