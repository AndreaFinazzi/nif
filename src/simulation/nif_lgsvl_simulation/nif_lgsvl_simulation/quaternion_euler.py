'''
@author Seungwook Lee
@date   2020-12-02
@brief  Convertions between Quaternion and Euler angles.
@source https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
'''
import numpy as np
import math
from geometry_msgs.msg import Vector3, Quaternion

class Quaternion_Euler(object):
    def __init__(self, q=None, e=None):
        if q is None:
            self.q = Quaternion()
        else:
            self.q = q
        if e is None:
            self.e = Vector3()
        else:
            self.e = e

    def ToQuaternion(self):
        cy = math.cos(self.e.z * 0.5)
        sy = math.sin(self.e.z * 0.5)
        cp = math.cos(self.e.y * 0.5)
        sp = math.sin(self.e.y * 0.5)
        cr = math.cos(self.e.x * 0.5)
        sr = math.sin(self.e.x * 0.5)

        self.q.w = cr * cp * cy + sr * sp * sy
        self.q.x = sr * cp * cy - cr * sp * sy
        self.q.y = cr * sp * cy + sr * cp * sy
        self.q.z = cr * cp * sy - sr * sp * cy

        self.q = self.normalizeQuaternion(self.q)

        return self.q

    def ToEuler(self):
        # roll (x-axis rotation)
        sinr_cosp = 2 * (self.q.w * self.q.x + self.q.y * self.q.z)
        cosr_cosp = 1 - 2 * (self.q.x * self.q.x + self.q.y * self.q.y)
        self.e.x = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (self.q.w * self.q.y - self.q.z * self.q.x)
        if abs(sinp) >= 1:
            self.e.y = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            self.e.y = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (self.q.w * self.q.z + self.q.x * self.q.y)
        cosy_cosp = 1 - 2 * (self.q.y * self.q.y + self.q.z * self.q.z)
        self.e.z = math.atan2(siny_cosp, cosy_cosp)

        return self.e

    def normalizeQuaternion(self, q):
        mag = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if mag == 0:
            q.w = 1
        else:
            q.x = q.x / mag
            q.y = q.y / mag
            q.z = q.z / mag
            q.w = q.w / mag

        return q
