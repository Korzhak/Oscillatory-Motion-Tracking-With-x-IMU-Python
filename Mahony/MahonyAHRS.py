"""
    Author: X-IO (https://x-io.co.uk/)
    Python implementer: Korzhak (GitHub)
    Date: 15.12.2022

    Filtering by Mahony method
"""

import numpy as np

GR = 9.81


class Mahony:
    def __init__(self, sample_freq=100, kp_def=1., ki_def=0.0):
        self.kp = kp_def
        self.ki = ki_def

        self.sample_freq = sample_freq
        self.sample_period = 1 / sample_freq

        self.Quaternion = np.array([1., 0., 0., 0.])

        self.e_int = np.array([0., 0., 0.])

    @staticmethod
    def prod_q(quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                         x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                         -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                         x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    @staticmethod
    def conjugate_quat(q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def Q_to_DCM(self):
        w, x, y, z = self.Quaternion
        return np.array([
            [1.0 - 2.0 * (y ** 2 + z ** 2), 2.0 * (x * y - w * z),
             2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x ** 2 + z ** 2),
             2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (w * x + y * z),
             1.0 - 2.0 * (x ** 2 + y ** 2)]])

    def update_marg(self, g: np.array, a: np.array, m: np.array) -> np.array:
        """
        Calculating quaternion using accelerometer, gyroscope and magnetometer data

        :param a: Accelerometer data [ax, ay, az]
        :param g: Gyroscope data [wx, wy, wz]
        :param m: Magnetometer data [mx, my, mz]
        :return: Calculated quaternion [w, i, j, k].
        """

        g = g.copy()
        a = a.copy()
        m = m.copy()

        q = self.Quaternion.copy()

        if np.linalg.norm(a) == 0 or np.linalg.norm(m) == 0:
            return self.Quaternion

        # Normalize accelerometer and magnetometer measurement
        a /= np.linalg.norm(a)
        m /= np.linalg.norm(a)

        # Reference direction of Earth's magnetic feild
        h = self.prod_q(q, self.prod_q(np.array([0, m[0], m[1], m[2]]), self.conjugate_quat(q)))
        b = np.array([0, np.linalg.norm(h[1:3]), 0, h[3]])

        # Estimated direction of gravity and magnetic field
        v = np.array([
            2 * (q[1] * q[3] - q[0] * q[2]),
            2 * (q[0] * q[1] + q[2] * q[3]),
            q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2
        ])

        w = np.array([
            2 * b[1] * (0.5 - q[2]**2 - q[3]**2) + 2 * b[3] * (q[1] * q[3] - q[0] * q[2]),
            2 * b[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * b[3] * (q[0] * q[1] + q[2] * q[3]),
            2 * b[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * b[3] * (0.5 - q[1]**2 - q[2]**2)
        ])

        # Error is sum of cross product between estimated direction and measured direction of fields
        e = np.cross(a, v) + np.cross(m, w)

        # Apply feedback terms
        if self.ki > 0:
            self.e_int += e * self.sample_period

        # Apply feedback terms
        g += self.kp * e + self.ki * self.e_int

        # Compute rate of change of quaternion
        q_dot = 0.5 * self.prod_q(q, np.array([0, g[0], g[1], g[2]]))

        # Integrate to yield quaternion
        q += q_dot * self.sample_period
        self.Quaternion = q / np.linalg.norm(q)
        return self.Quaternion

    def update_imu(self, g: np.array, a: np.array) -> np.array:
        """
        Calculating quaternion using accelerometer and gyroscope data

        :param g: Gyroscope data [wx, wy, wz]
        :param a: Accelerometer data [ax, ay, az]
        :return: Calculated quaternion [w, i, j, k].
        """

        q = self.Quaternion.copy()

        g = g.copy()
        a = a.copy()

        if np.linalg.norm(a) == 0:
            return self.Quaternion

        a /= np.linalg.norm(a)

        # Estimated direction of gravity and vector perpendicular to magnetic flux
        v = np.array([
            2 * (q[1] * q[3] - q[0] * q[2]),
            2 * (q[0] * q[1] + q[2] * q[3]),
            q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2
        ])

        # Error is sum of cross product between estimated and measured direction of gravity
        e = np.cross(a, v)

        # Compute and apply integral feedback if enabled
        if self.ki > 0:
            self.e_int += e * self.sample_period

        # Apply feedback terms
        g += self.kp * e + self.ki * self.e_int

        # Compute rate of change of quaternion
        q_dot = 0.5 * self.prod_q(q, np.array([0, g[0], g[1], g[2]]))

        q += q_dot * self.sample_period
        self.Quaternion = q / np.linalg.norm(q)
        return self.Quaternion
