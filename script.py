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
from scipy.signal import butter, filtfilt
from ahrs.common import Quaternion
from ahrs.filters import Mahony
from ximu_python_library import xIMUdataClass as xIMU


file_path = 'logged_data/LoggedData'
sample_rate = 256
sample_period = 1 / sample_rate


# Import data
xIMUdata = xIMU.xIMUdataClass(file_path, sr=sample_rate)

gyr = np.array([xIMUdata.CalInertialAndMagneticData.gyroscope[:, 0],
                xIMUdata.CalInertialAndMagneticData.gyroscope[:, 1],
                xIMUdata.CalInertialAndMagneticData.gyroscope[:, 2]]).T

acc = np.array([xIMUdata.CalInertialAndMagneticData.accelerometer[:, 0],
                xIMUdata.CalInertialAndMagneticData.accelerometer[:, 1],
                xIMUdata.CalInertialAndMagneticData.accelerometer[:, 2]]).T

# Plot
# TODO: plotting data

# Process data through AHRS algorithm (calculate orientation)
# See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Q = Quaternion(np.array([1., 0., 0., 0.]))  # AHRS calculated Quaternion
R = np.zeros((3, 3, af.length(gyr)))        # rotation matrix describing sensor relative to Earth

attitude = Mahony(frequency=sample_rate)

for i in range(af.length(gyr)):
    Q = attitude.updateIMU(Q, gyr[i, :], acc[i, :])  # gyroscope units must be radians
    R[:, :, i] = Quaternion(Q).to_DCM().T  # transpose because ahrs provides Earth relative to sensor

# Calculate 'tilt-compensated' accelerometer
tc_acc = np.zeros(acc.shape)  # accelerometer in Earth frame

for i in range(af.length(acc)):
    tc_acc[i, :] = R[:, :, i] @ acc[i, :].T  # product rotation matrix by transpose acceleration

# Plot
# TODO: plotting data

# Calculate linear acceleration in Earth frame (subtracting gravity)
lin_acc = tc_acc - np.array([np.zeros(af.length(tc_acc)),
                             np.zeros(af.length(tc_acc)),
                             np.ones(af.length(tc_acc))]).T

lin_acc *= 9.81  # convert from 'g' to m/s^2

# Plot
# TODO: plotting data

# Calculate linear velocity (integrate acceleration)
lin_vel = np.zeros(lin_acc.shape)

for i in range(1, af.length(lin_acc)):
    lin_vel[i, :] = lin_vel[i-1, :] + lin_acc[i, :] * sample_period

# Plot
# TODO: plotting data

# High-pass filter linear velocity to remove drift
order = 1
filter_cut_off = 0.1
[b, a] = butter(order, (2*filter_cut_off)/sample_rate, 'high')
lin_vel_hp = filtfilt(b, a, lin_vel.T).T

# Plot
# TODO: plotting data

# Calculate linear position (integrate velocity)
lin_pos = np.zeros(lin_vel_hp.shape)

for i in range(1, af.length(lin_vel_hp)):
    lin_pos[i, :] = lin_pos[i-1, :] + lin_vel_hp[i, :] * sample_period

# Plot
# TODO: plotting data

# High-pass filter linear position to remove drift
lin_pos_hp = filtfilt(b, a, lin_pos.T).T

# Plot
# TODO: plotting data

print(lin_pos_hp)
