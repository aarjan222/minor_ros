import math


class Quaternion:
    def __init__(self, x, y, z, w):
        self.w = w
        self.x = x
        self.y = y
        self.z = z


class Euler:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


def quat_to_euler(q):
    euler = Euler(0, 0, 0)

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x**2 + q.y**2)
    euler.roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = math.sqrt(1 + 2 * (q.w * q.y + q.x * q.z))
    cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    euler.pitch = 2*math.atan2(sinp, cosp) - (math.pi/2)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
    euler.yaw = math.atan2(siny_cosp, cosy_cosp)

    return euler.roll*180/math.pi, euler.pitch*180/math.pi, euler.yaw*180/math.pi


quat = Quaternion(0.14, -0.24, 0.03, 0.95)

print(quat_to_euler(quat))
