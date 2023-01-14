import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl

pg.mkQApp()

view = gl.GLViewWidget()
# view.setBackgroundColor("w")
view.setCameraPosition(distance=0.6)
view.show()

line_len = 0.02

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

x_point1 = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple
x_point2 = np.array([line_len, 0, 0])

y_point1 = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple
y_point2 = np.array([0, line_len, 0])

z_point1 = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple
z_point2 = np.array([0, 0, line_len])  # specify the (x, y, z) values of the second point in a tuple

x_axis = gl.GLLinePlotItem(pos=np.array([x_point1, x_point2]), width=3)
y_axis = gl.GLLinePlotItem(pos=np.array([y_point1, y_point2]), width=3)
z_axis = gl.GLLinePlotItem(pos=np.array([z_point1, z_point2]), width=3)
x_axis.setData(color=(1, 0, 0, 1))
y_axis.setData(color=(0, 1, 0, 1))
z_axis.setData(color=(0, 0, 1, 1))

view.addItem(x_axis)
view.addItem(y_axis)
view.addItem(z_axis)
lin_pos_hp = np.load("lin_pos_hp.npy")
R = np.load("rot_mat.npy")

i = 0


def length(array: np.array) -> int:
    """
    Like a length() in Matlab.

    :param array: numpy array.
    :return: the length of the largest array dimension.
    """
    return max(array.shape)


def update():
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
    if i <= length(lin_pos_hp):
        pos = lin_pos_hp[i, :].copy()
        a = pos.copy()
        b = R[:, 0, i].copy()
        l = np.linalg.norm(b - a)
        c = b * line_len / l
        x_point1 = pos
        x_point2 = pos + c
        x_axis.setData(pos=np.array([x_point1, x_point2]))

        a = pos.copy()
        b = R[:, 1, i].copy()
        l = np.linalg.norm(b - a)
        c = b * line_len / l
        y_point1 = pos
        y_point2 = pos + c
        y_axis.setData(pos=np.array([y_point1, y_point2]))

        a = pos.copy()
        b = R[:, 2, i].copy()
        l = np.linalg.norm(b - a)
        c = b * line_len / l
        z_point1 = pos
        z_point2 = pos + c
        z_axis.setData(pos=np.array([z_point1, z_point2]))
        i += 10


if __name__ == '__main__':
    t = QtCore.QTimer()
    t.timeout.connect(update)
    t.start(1)
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QGuiApplication.instance().exec_()
