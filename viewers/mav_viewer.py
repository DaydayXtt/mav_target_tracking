"""
mavsim_python: mav viewer (for chapter 2)
    - Beard & McLain, PUP, 2012
    - Update history:
        1/15/2019 - RWB
        4/15/2019 - BGM
        3/31/2020 - RWB
        7/13/2023 - RWB
        3/25/2024 - Carson Moon
"""
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
import pyqtgraph.Qt.QtGui as QtGui
# from viewers.draw_mav import DrawMav
from viewers.draw_mav_stl import DrawMav
from time import time

class MavViewer():
    def __init__(self, app, ts_refresh=1./30.):
        self.scale = 10
        # initialize Qt gui application and window
        self.app = app  # initialize QT, external so that only one QT process is running
        self.window = gl.GLViewWidget()  # initialize the view object
        #gl.GLViewWidget.getViewport().setAttribute(QtCore.Qt.WidgetAttribute.WA_AcceptTouchEvents, False)
        self.window.setWindowTitle('MAV Viewer')
        grid = gl.GLGridItem(size=QtGui.QVector3D(1000,1000,1),
                             color=(125, 125, 125, 76.5)) # make a grid to represent the ground
        grid.scale(self.scale, self.scale, self.scale) # set the size of the grid (distance between each line)
        grid.size() # set the number of lines in the grid
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=200) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.setGeometry(0, 0, 750, 750)  # args: upper_left_x, upper_right_y, width, height
        # center = self.window.cameraPosition()
        # center.setX(0)
        # center.setY(0)
        # center.setZ(0)
        # self.window.setCameraPosition(pos=center, distance=self.scale, elevation=50, azimuth=-45)
        # self.window.setCameraPosition(pos=center, distance=300, elevation=50, azimuth=45)  # 调整视图大小

#        self.window.viewport().setAttribute(QtCore.Qt.WidgetAttribute.WA_AcceptTouchEvents, False)
        self.window.show()  # display configured window
        # self.window.raise_() # bring window to the front

        self.plot_initialized = False # has the mav been plotted yet?
        self.mav_plot = []
        self.ts_refresh = ts_refresh
        self.t = time()
        self.t_next = self.t       

    def update(self, state):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.mav_plot = DrawMav(state, self.window, scale=self.scale)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            t = time()
            if t-self.t_next > 0.0:
                self.mav_plot.update(state)
                self.t = t
                self.t_next = t + self.ts_refresh
        # update the center of the camera view to the mav location
        view_location = Vector(state.east, state.north, state.altitude)  # defined in ENU coordinates
        # view_location = Vector(0, 0, 0)  # 固定视角
        self.window.opts['center'] = view_location
        # redraw
    
    def process_app(self):
        self.app.processEvents()

    def clear_viewer(self):
        self.window.clear()

