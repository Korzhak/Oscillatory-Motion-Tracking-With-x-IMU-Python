# encoding: utf-8
#
#                    Oscillatory-Motion-Tracking-With-x-IMU-Python
# This project is realization of Oscillatory-Motion-Tracking-With-x-IMU on python.
# Link to origin code: https://github.com/xioTechnologies/Oscillatory-Motion-Tracking-With-x-IMU
#  Date: 10.01.2023
#  Author: Korzhak (GitHub)
#  Ukraine
#

import numpy as np
import additional_func as af
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import butter, filtfilt
from ahrs.common import Quaternion
from ahrs.filters import Mahony
from ximu_python_library import xIMUdataClass as xIMU

file_path = 'logged_data/LoggedData'
sample_rate = 256
sample_period = 1 / sample_rate

# Import data

xIMUdata = xIMU.xIMUdataClass(file_path, sr=sample_rate)

time = xIMUdata.CalInertialAndMagneticData.Time

gyr = np.array([xIMUdata.CalInertialAndMagneticData.gyroscope[:, 0],
                xIMUdata.CalInertialAndMagneticData.gyroscope[:, 1],
                xIMUdata.CalInertialAndMagneticData.gyroscope[:, 2]]).T

acc = np.array([xIMUdata.CalInertialAndMagneticData.accelerometer[:, 0],
                xIMUdata.CalInertialAndMagneticData.accelerometer[:, 1],
                xIMUdata.CalInertialAndMagneticData.accelerometer[:, 2]]).T

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(time, acc[:, 0], c='r', linewidth=0.5)
plt.plot(time, acc[:, 1], c='g', linewidth=0.5)
plt.plot(time, acc[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("acceleration")
plt.xlabel("time (s)")
plt.ylabel("g")
plt.show(block=False)

fig = plt.figure(figsize=(10, 5))
plt.plot(time, gyr[:, 0], c='r', linewidth=0.5)
plt.plot(time, gyr[:, 1], c='g', linewidth=0.5)
plt.plot(time, gyr[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("gyroscope")
plt.xlabel("time (s)")
plt.ylabel("rad/sec")
plt.show(block=False)

# Process data through AHRS algorithm (calculate orientation)
# See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Q = Quaternion(np.array([1., 0., 0., 0.]))  # AHRS calculated Quaternion
R = np.zeros((3, 3, af.length(gyr)))  # rotation matrix describing sensor relative to Earth

attitude = Mahony(frequency=sample_rate)

for i in range(af.length(gyr)):
    Q = attitude.updateIMU(Q, gyr[i, :], acc[i, :])  # gyroscope units must be radians
    R[:, :, i] = Quaternion(Q).to_DCM().T  # transpose because ahrs provides Earth relative to sensor

# Calculate 'tilt-compensated' accelerometer

tc_acc = np.zeros(acc.shape)  # accelerometer in Earth frame

for i in range(af.length(acc)):
    tc_acc[i, :] = R[:, :, i] @ acc[i, :].T  # product rotation matrix by transpose acceleration

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(time, tc_acc[:, 0], c='r', linewidth=0.5)
plt.plot(time, tc_acc[:, 1], c='g', linewidth=0.5)
plt.plot(time, tc_acc[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Transpose acceleration")
plt.xlabel("time (s)")
plt.ylabel("g")
plt.show(block=False)

# Calculate linear acceleration in Earth frame (subtracting gravity)

lin_acc = tc_acc - np.array([np.zeros(af.length(tc_acc)),
                             np.zeros(af.length(tc_acc)),
                             np.ones(af.length(tc_acc))]).T

lin_acc *= 9.81  # convert from 'g' to m/s^2

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(time, lin_acc[:, 0], c='r', linewidth=0.5)
plt.plot(time, lin_acc[:, 1], c='g', linewidth=0.5)
plt.plot(time, lin_acc[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Linear acceleration")
plt.xlabel("time (s)")
plt.ylabel("m/s^2")
plt.show(block=False)

# Calculate linear velocity (integrate acceleration)

lin_vel = np.zeros(lin_acc.shape)

for i in range(1, af.length(lin_acc)):
    lin_vel[i, :] = lin_vel[i - 1, :] + lin_acc[i, :] * sample_period

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(time, lin_vel[:, 0], c='r', linewidth=0.5)
plt.plot(time, lin_vel[:, 1], c='g', linewidth=0.5)
plt.plot(time, lin_vel[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Linear velocityt")
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
plt.plot(time, lin_vel_hp[:, 0], c='r', linewidth=0.5)
plt.plot(time, lin_vel_hp[:, 1], c='g', linewidth=0.5)
plt.plot(time, lin_vel_hp[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("High-pass filtered linear velocity")
plt.xlabel("time (s)")
plt.ylabel("m/s")
plt.show(block=False)

# Calculate linear position (integrate velocity)

lin_pos = np.zeros(lin_vel_hp.shape)

for i in range(1, af.length(lin_vel_hp)):
    lin_pos[i, :] = lin_pos[i - 1, :] + lin_vel_hp[i, :] * sample_period

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(time, lin_pos[:, 0], c='r', linewidth=0.5)
plt.plot(time, lin_pos[:, 1], c='g', linewidth=0.5)
plt.plot(time, lin_pos[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("Linear position")
plt.xlabel("time (s)")
plt.ylabel("m")
plt.show(block=False)

# High-pass filter linear position to remove drift

lin_pos_hp = filtfilt(b, a, lin_pos.T).T

# Plot

fig = plt.figure(figsize=(10, 5))
plt.plot(time, lin_pos_hp[:, 0], c='r', linewidth=0.5)
plt.plot(time, lin_pos_hp[:, 1], c='g', linewidth=0.5)
plt.plot(time, lin_pos_hp[:, 2], c='b', linewidth=0.5)
plt.legend(["x", "y", "z"])
plt.title("High-pass filtered linear position")
plt.xlabel("time (s)")
plt.ylabel("m")
plt.show(block=False)

# Create 6 DOF animation
# plt.ion()
# fig = plt.figure(figsize=(7, 7))
# ax = fig.add_subplot(111, projection='3d')  # Axe3D object
# point, = ax.plot(lin_pos_hp[0, 0], lin_pos_hp[0, 1], lin_pos_hp[0, 2], marker='o')
# min_, max_ = np.min(np.min(lin_pos_hp, axis=0)), np.max(np.max(lin_pos_hp, axis=0))
# ax.set_xlim(min_, max_)
# ax.set_ylim(min_, max_)
# ax.set_zlim(min_, max_)
# ax.set_title("trajectory")
# ax.set_xlabel("x position (m)")
# ax.set_ylabel("y position (m)")
# ax.set_zlabel("z position (m)")
#
plt.show()
#
# for i in range(af.length(lin_pos_hp)):
#     point.set_xdata([lin_pos_hp[i, 0]])
#     point.set_ydata([lin_pos_hp[i, 1]])
#     point.set_3d_properties([lin_pos_hp[i, 2]])
#     plt.pause(sample_period)
