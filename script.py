# -*- coding: utf-8 -*-
#
#                    Oscillatory-Motion-Tracking-With-x-IMU-Python
# This project is realization of Oscillatory-Motion-Tracking-With-x-IMU on python.
# Link to origin code: https://github.com/xioTechnologies/Oscillatory-Motion-Tracking-With-x-IMU
#  Date: 10.01.2023
#  Author: Korzhak (GitHub)
#  Ukraine
#

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from Mahony.MahonyAHRS import Mahony
from ximu_python_library import xIMUdataClass as xIMU


# Additional functions

def length(array: np.array) -> int:
    """
    Like a length() in Matlab.

    :param array: numpy array.
    :return: the length of the largest array dimension.
    """
    return max(array.shape)


# Main settings

file_path = 'logged_data/LoggedData'
sample_rate = 256
sample_period = 1 / sample_rate

# Import data

xIMUdata = xIMU.xIMUdataClass(file_path, sr=sample_rate)

x_time = xIMUdata.CalInertialAndMagneticData.Time

gyr = np.array([xIMUdata.CalInertialAndMagneticData.gyroscope[:, 0],
                xIMUdata.CalInertialAndMagneticData.gyroscope[:, 1],
                xIMUdata.CalInertialAndMagneticData.gyroscope[:, 2]]).T

acc = np.array([xIMUdata.CalInertialAndMagneticData.accelerometer[:, 0],
                xIMUdata.CalInertialAndMagneticData.accelerometer[:, 1],
                xIMUdata.CalInertialAndMagneticData.accelerometer[:, 2]]).T

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, acc[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, acc[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, acc[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("acceleration")
plt.xlabel("time (s)")
plt.ylabel("g")
plt.show(block=False)

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, gyr[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, gyr[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, gyr[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("gyroscope")
plt.xlabel("time (s)")
plt.ylabel("rad/sec")
plt.show(block=False)

# Process data through AHRS algorithm (calculate orientation)
# See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = np.zeros((3, 3, length(gyr)))  # rotation matrix describing sensor relative to Earth

ahrs = Mahony(sample_freq=sample_rate, ki_def=0)
np.set_printoptions(suppress=True)

for i in range(length(gyr)):
    ahrs.update_imu(gyr[i, :].copy() * (np.pi/180), acc[i, :].copy())  # gyroscope units must be radians
    R[:, :, i] = ahrs.Q_to_DCM()                         # transpose because ahrs provides Earth relative to sensor

np.save("rot_mat.npy", R)

# Calculate 'tilt-compensated' accelerometer

tc_acc = np.zeros(acc.shape)  # accelerometer in Earth frame

for i in range(length(acc)):
    tc_acc[i, :] = R[:, :, i] @ acc[i, :]  # product rotation matrix by transpose acceleration

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, tc_acc[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, tc_acc[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, tc_acc[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("'Tilt-compensated' accelerometer")
plt.xlabel("time (s)")
plt.ylabel("g")
plt.show(block=False)

# Calculate linear acceleration in Earth frame (subtracting gravity)

lin_acc = tc_acc - np.array([np.zeros(length(tc_acc)),
                             np.zeros(length(tc_acc)),
                             np.ones(length(tc_acc))]).T

lin_acc *= 9.81  # convert from 'g' to m/s^2

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, lin_acc[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, lin_acc[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, lin_acc[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Linear acceleration")
plt.xlabel("time (s)")
plt.ylabel("m/s^2")
plt.show(block=False)

# Calculate linear velocity (integrate acceleration)

lin_vel = np.zeros(lin_acc.shape)

for i in range(1, length(lin_acc)):
    lin_vel[i, :] = lin_vel[i - 1, :] + lin_acc[i, :] * sample_period

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, lin_vel[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, lin_vel[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, lin_vel[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Linear velocity")
plt.xlabel("time (s)")
plt.ylabel("m/s")
plt.show(block=False)

# High-pass filter linear velocity to remove drift

order = 1
filter_cut_off = 0.1
[b, a] = butter(order, (2 * filter_cut_off) / sample_rate, 'high')
lin_vel_hp = filtfilt(b, a, lin_vel.T).T

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, lin_vel_hp[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, lin_vel_hp[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, lin_vel_hp[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("High-pass filtered linear velocity")
plt.xlabel("time (s)")
plt.ylabel("m/s")
plt.show(block=False)

# Calculate linear position (integrate velocity)

lin_pos = np.zeros(lin_vel_hp.shape)

for i in range(1, length(lin_vel_hp)):
    lin_pos[i, :] = lin_pos[i - 1, :] + lin_vel_hp[i, :] * sample_period

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, lin_pos[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, lin_pos[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, lin_pos[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Linear position")
plt.xlabel("time (s)")
plt.ylabel("m")
plt.show(block=False)

# High-pass filter linear position to remove drift

lin_pos_hp = filtfilt(b, a, lin_pos.T).T

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(x_time, lin_pos_hp[:, 0], c='r', linewidth=0.5)
plt.plot(x_time, lin_pos_hp[:, 1], c='g', linewidth=0.5)
plt.plot(x_time, lin_pos_hp[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("High-pass filtered linear position")
plt.xlabel("time (s)")
plt.ylabel("m")
plt.show()

np.save("lin_pos_hp.npy", lin_pos_hp)
