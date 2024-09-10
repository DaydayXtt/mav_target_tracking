"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import numpy as np
import pyqtgraph.opengl as gl


class DrawLine:
    def __init__(self, line: np.ndarray, color: np.ndarray, window: gl.GLViewWidget):
        self.color = color
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = line @ R.T
        path_color = np.tile(color, (points.shape[0], 1))

        self.path_plot_object = gl.GLLinePlotItem(pos=points,
                                                  color=path_color,
                                                  width=1,
                                                  antialias=True,
                                                  mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, line: np.ndarray, color: np.ndarray):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = line @ R.T
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object.setData(pos=points, color=path_color)

