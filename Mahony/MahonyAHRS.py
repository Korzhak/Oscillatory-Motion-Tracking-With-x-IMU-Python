"""
    Автор: X-IO (https://x-io.co.uk/)
    Реалізація: Korzhak (GitHub)
    Дата: 15.12.2022

    Модуль для фільтрування даних методом Магоні.
"""

from math import sqrt, atan2, asin
import numpy as np

GR = 9.81


class Mahony:
    def __init__(self, sample_freq=100, kp_def=1., ki_def=0.0):
        """
        Ініціалізація.

        :param sample_freq: частота оновлення даних (в ГЦ).
        :param kp_def: TODO: дописати доку
        :param ki_def: TODO: дописати доку
        """
        self.kp = kp_def
        self.ki = ki_def

        self.sample_freq = sample_freq
        self.sample_period = 1 / sample_freq

        self.Quaternion = np.array([1., 0., 0., 0.])

        self.e_int = np.array([0., 0., 0.])

    @staticmethod
    def quaternion_multiply(quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                         x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                         -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                         x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    def Q_to_DCM(self):
        w, x, y, z = self.Quaternion
        return np.array([
            [1.0 - 2.0 * (y ** 2 + z ** 2), 2.0 * (x * y - w * z),
             2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x ** 2 + z ** 2),
             2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (w * x + y * z),
             1.0 - 2.0 * (x ** 2 + y ** 2)]])
    
    def update_imu(self, g: np.array, a: np.array) -> np.array:
        """
        Фільтрування даних методом Магоні.

        :param a:
        :param g:
        :return: Розрахований кватерніон (w, i, j, k).
        """

        q = self.Quaternion.copy()

        if a.sum() == 0:
            return self.Quaternion

        norma = np.linalg.norm(a)
        a /= norma

        # Estimated direction of gravity and vector perpendicular to magnetic flux
        v = np.array([
            2 * (q[1] * q[3] - q[0] * q[2]),
            2 * (q[0] * q[1] + q[2] * q[3]),
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
        ])

        # Error is sum of cross product between estimated and measured direction of gravity
        e = np.cross(a, v)

        # Compute and apply integral feedback if enabled
        if self.ki > 0:
            self.e_int += e * self.sample_period

        # Apply feedback terms
        g += self.kp * e + self.ki * self.e_int

        # Compute rate of change of quaternion
        q_dot = 0.5 * self.quaternion_multiply(q, np.array([0, g[0], g[1], g[2]]))

        q += q_dot * self.sample_period

        self.Quaternion = q / np.linalg.norm(q)
        return self.Quaternion
