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
from ahrs.filters import Mahony
from ahrs.common import Quaternion

# Import data

gyr = np.zeros((100, 3))  # 100 is amount of gyr data for each axes
acc = np.zeros((100, 3))  # 100 is amount of gyr data for each axes

Q = Quaternion(np.array([1., 0., 0., 0.]))     # AHRS calculated Quaternion
Q_pr = Quaternion(np.array([1., 0., 0., 0.]))  # preview Quaternion

# Plot

# TODO: plotting data


# Process data through AHRS algorithm (calculate orientation)
# See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = np.zeros((3, 3, af.length(gyr)))  # rotation matrix describing sensor relative to Earth

mahony_filter = Mahony(frequency=100)

for i in range(af.length(gyr)):
    Q = mahony_filter.updateIMU(Q_pr, gyr[i, :], acc[i, :])  # gyroscope units must be radians
    R[:, :, i] = Quaternion(Q).to_DCM().T                    # transpose because ahrs provides Earth relative to sensor
    Q_pr = Q


# Calculate 'tilt-compensated' accelerometer

tc_acc = np.zeros(acc.shape)  # accelerometer in Earth frame

for i in range(af.length(acc)):
    tc_acc[i, :] = R[:, :, i] @ acc[i, :].T  # product rotation matrix by transpose acceleration


