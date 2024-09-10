"""
mavsim_python: world viewer (for chapter 12)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
"""
import numpy as np
from viewers.draw_mav_stl import DrawMav
from viewers.draw_path import DrawPath
from viewers.draw_waypoints import DrawWaypoints
from viewers.draw_map import DrawMap
from viewers.draw_target import DrawTarget
from viewers.draw_line import DrawLine

import pyqtgraph.opengl as gl
import pyqtgraph.Qt.QtGui as QtGui


class MAVWorldViewer:
    def __init__(self, app):
        self.scale = 250
        # initialize Qt gui application and window
        self.app = app  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('World Viewer')
        self.window.setGeometry(0, 0, 1600, 1000)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem(size=QtGui.QVector3D(1000,1000,1),
                             color=(200, 200, 200, 76.5)) # make a grid to represent the ground
        grid.scale(self.scale/10, self.scale/10, self.scale/10) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        center = self.window.cameraPosition()
        center.setX(1000)
        center.setY(1000)
        center.setZ(0)
        self.window.setCameraPosition(pos=center, distance=3000, elevation=50, azimuth=-90)
        self.window.setBackgroundColor('gray')  # set background color to black
        # self.window.resize(*(4000, 4000))  # not sure how to resize window
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False  # has the mav been plotted yet?
        self.mav_plot = []
        self.path_plot = []
        self.waypoint_plot = []
        self.map_plot = []
        self.target_plot = []
        self.line_plot = []

    def update(self, state, path, map,
               target_position: np.ndarray,
               voro
               ):
        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[1., 0., 0., 1]])
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            # for vnoicell in voro:
            #     vertices = np.array(vnoicell['vertices'])
            #     for face in vnoicell['faces']:
            #         for i in range(len(face['vertices'])):
            #             if i == len(face['vertices'])-1:
            #                 p1 = vertices[face['vertices'][i]]
            #                 p2 = vertices[face['vertices'][0]]
            #                 line = np.array([p1, p2])
            #             else:
            #                 p1 = vertices[face['vertices'][i]]
            #                 p2 = vertices[face['vertices'][i+1]]
            #                 line = np.array([p1, p2])
            #             self.line_plot.append(DrawLine(line, red, self.window))
            self.map_plot = DrawMap(map, self.window)
            self.path_plot = DrawPath(path, red, self.window)
            self.mav_plot = DrawMav(state, self.window, scale=5)
            for i in range(target_position.shape[0]):
                target = target_position[[i]].T
                self.target_plot.append(DrawTarget(target, self.window))
            # self.target_plot = DrawTarget(target_position, self.window)

            self.plot_initialized = True
            path.plot_updated = True
        # else update drawing on all other calls to update()
        else:
            self.mav_plot.update(state)
            self.path_plot.update(path, red)
            # for tar in self.target_plot:
            for i in range(target_position.shape[0]):
                target = target_position[[i]]
                tar = self.target_plot[i]
                tar.update(target)

            # for line_v in self.line_plot:
            #     for vnoicell in voro:
            #         vertices = np.array(vnoicell['vertices'])
            #         for face in vnoicell['faces']:
            #             for i in range(len(face['vertices'])):
            #                 if i == len(face['vertices'])-1:
            #                     p1 = vertices[face['vertices'][i]]
            #                     p2 = vertices[face['vertices'][0]]
            #                     line = np.array([p1, p2])
            #                 else:
            #                     p1 = vertices[face['vertices'][i]]
            #                     p2 = vertices[face['vertices'][i+1]]
            #                     line = np.array([p1, p2])
            #                 line_v.update(line, red)


            # if not path.plot_updated:
            #     self.path_plot.update(path, red)
            #     path.plot_updated = True
        # redraw
        self.app.processEvents()
