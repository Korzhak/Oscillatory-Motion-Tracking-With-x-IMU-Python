# -*- coding: utf-8 -*-
#
#              SixDOFAnimation
# Implementation of 6D animation as in the
# Oscillatory-Motion-Tracking-With-x-IMU project.
# https://www.youtube.com/watch?v=SI1w9uaBw6Q
# For high performance, the PyQtGraph library is used.
#  Date: 14.01.2023
#  Author: Korzhak (GitHub)
#  Ukraine
#

import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl

pg.mkQApp()

# Make view in window

view = gl.GLViewWidget()
view.setCameraPosition(distance=0.6)
view.show()

# Set vectors length

vector_len = 0.1

# Making grid surfaces in 3 axes

gx = gl.GLGridItem()
gx.setSize(x=0.2, y=0.2, z=0.2)
gx.setSpacing(x=0.01, y=0.01, z=0.01)
gx.rotate(90, 0, 1, 0)
gx.translate(-0.1, 0, 0)
view.addItem(gx)

gy = gl.GLGridItem()
gy.setSize(x=0.2, y=0.2, z=0.2)
gy.setSpacing(x=0.01, y=0.01, z=0.01)
gy.rotate(90, 1, 0, 0)
gy.translate(0, 0.1, 0)
view.addItem(gy)

gz = gl.GLGridItem()
gz.setSize(x=0.2, y=0.2, z=0.2)
gz.setSpacing(x=0.01, y=0.01, z=0.01)
gz.translate(0, 0, -0.1)
view.addItem(gz)

# Making 3 vectors in space

x_point1 = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple
x_point2 = np.array([vector_len, 0, 0])

y_point1 = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple
y_point2 = np.array([0, vector_len, 0])

z_point1 = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple
z_point2 = np.array([0, 0, vector_len])  # specify the (x, y, z) values of the second point in a tuple

x_axis = gl.GLLinePlotItem(pos=np.array([x_point1, x_point2]), width=3)
y_axis = gl.GLLinePlotItem(pos=np.array([y_point1, y_point2]), width=3)
z_axis = gl.GLLinePlotItem(pos=np.array([z_point1, z_point2]), width=3)

x_axis.setData(color=(1, 0, 0, 1))
y_axis.setData(color=(0, 1, 0, 1))
z_axis.setData(color=(0, 0, 1, 1))

view.addItem(x_axis)
view.addItem(y_axis)
view.addItem(z_axis)

# Load position data and rotation vectors data

lin_pos_hp = np.load("logged_data/lin_pos_hp.npy")
R = np.load("logged_data/rot_mat.npy")

i = 0
step = 10


def update():
    """
    Function which update position of vectors in space
    """
    global x_axis
    global x_point1
    global x_point2

    global y_axis
    global y_point1
    global y_point2

    global z_axis
    global z_point1
    global z_point2

    global lin_pos_hp
    global R
    global i
    if i <= max(lin_pos_hp.shape):

        # Set X vector
        pos = lin_pos_hp[i, :].copy()
        a = pos.copy()
        b = R[:, 0, i].copy()
        l = np.linalg.norm(b - a)    # length of vector
        c = b * vector_len / l       # find new vector with length 'vector_len'
        x_point1 = pos               # set start point
        x_point2 = pos + c           # set end point
        x_axis.setData(pos=np.array([x_point1, x_point2]))

        # Set Y vector
        a = pos.copy()
        b = R[:, 1, i].copy()
        l = np.linalg.norm(b - a)    # length of vector
        c = b * vector_len / l       # find new vector with length 'vector_len'
        y_point1 = pos               # set start point
        y_point2 = pos + c           # set end point
        y_axis.setData(pos=np.array([y_point1, y_point2]))

        # Set Z vector
        a = pos.copy()
        b = R[:, 2, i].copy()
        l = np.linalg.norm(b - a)    # length of vector
        c = b * vector_len / l       # find new vector with length 'vector_len'
        z_point1 = pos               # set start point
        z_point2 = pos + c           # set end point
        z_axis.setData(pos=np.array([z_point1, z_point2]))
        i += step


if __name__ == '__main__':
    t = QtCore.QTimer()
    t.timeout.connect(update)
    t.start(1)
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QGuiApplication.instance().exec_()
