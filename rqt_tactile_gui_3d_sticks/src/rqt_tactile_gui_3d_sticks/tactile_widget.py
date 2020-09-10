import math
import rospy
import numpy
from python_qt_binding.QtCore import QPoint, Qt, QTimer
from python_qt_binding.QtOpenGL import QGLFormat, QGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
OpenGL.ERROR_CHECKING = True


from tf.transformations import quaternion_matrix, quaternion_about_axis

import threading
import matplotlib
import matplotlib.pyplot

class TactileWidget(QGLWidget):

    def __init__(self, text_info = '', parent=None):
        glformat = QGLFormat()
        glformat.setSampleBuffers(True)
        super(TactileWidget, self).__init__(glformat, parent)
        self.setCursor(Qt.OpenHandCursor)
        self.setMouseTracking(True)
        self._modelview_matrix = numpy.identity(4)
        self._near = 0.1
        self._far = 100.0
        self._fovy = 45.0
        self._radius = 5.0
        self._last_point_2d = QPoint()
        self._last_point_3d = [0.0, 0.0, 0.0]
        self._last_point_3d_ok = False
        self._text_info = text_info

        """Define if the max values are calculated for each electrodes"""
        self._calibrationDone  = False

        """Set the max value of electrodes model file"""
        self._max_reading = 3700

        """Set a color array (for the 4096 values) by using a color map.
        The color array list for each value (0 to 4096) contains a sub list [r,g,b,a].
        """
        self.__color_map = matplotlib.cm.ScalarMappable(cmap='jet')
        self.__color_array = numpy.linspace(0,1,self._max_reading+1)
        self.__color_array = self.__color_map.to_rgba(self.__color_array)

        """Set the number of electrodes"""
        self._num_electrodes = 19

        """set the current values of 19 electrodes"""
        self._values = (0) * self._num_electrodes

        """set the maximum values of 19 electrodes"""
        self._maxValues = [0] * self._num_electrodes

        """Set color List of electrodes"""
        self._red = [255] *  self._num_electrodes
        self._green = [255] *  self._num_electrodes
        self._blue = [255] *  self._num_electrodes

        """Set Index position of 3 sticks to make a triangle. This list contains all triangles necessary for drawing the surface"""
        """Indexes in this list start with 1, the list of x and y positions of stick start with 0"""
        self._coord_triangles = [(11, 7, 9),(8, 9, 7),(1, 8, 7),(10, 11, 9),(10, 9, 8),(1, 10, 8),(12,11,10),(17,12,10),(2,17,10),(2,10,1),(14,11,12),(13,14,12),(13,12,17),(3,13,17),(3,17,2),(4,3,2),(4,2,1),(18,13,3),(18,15,13),(15,14,13),(5,18,3),(4,5,3),(6,5,4),(6,19,5),(5,19,18),(19,15,18),(19,16,15),(16,14,15)]

        """Array required to draw the height values of each electrodes. 1 height value per electrode"""
        self._height_values = [0] * self._num_electrodes

        """a threading lock is enable to concurrent data access """
        self._lock = threading.Lock()

        """Array of X and Y 2D positions of electrodes"""
        self._xPositions = []
        self._yPositions = []

        """Set the X and Y 2D positions of electrodes"""
        self._set_electrode_positions()

        """Default position and rotation in the 3D world"""
        self._position = (0.0, 0.0, 0.0)
        self._orientation = quaternion_about_axis(0.0, (1.0, 0.0, 0.0))
        self.setAcceptDrops(True)
        self._set_default_view()

        """A value to count the number of 'update color' message."""
        """10 messages before, the maximum values of each electrodes are calculated,"""
        """and stored in the _maxValues array"""
        self._nbUpdateColor = 0

        """Options for drawing"""
        self._drawMesh = False    # draw only mesh (lines)
        self._drawTriangles = False    # draw only colored triangles (surface)
        self._drawSticks = True    # draw only colored sticks

        """A Timer to update the graphic each 100 ms"""
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.updateGL)
        self._timer.start(40)

    """Set drawing Options"""
    def setDrawMesh(self, value):
        self._drawMesh = value

    def setDrawTriangles(self, value):
        self._drawTriangles = value

    def setDrawStricks(self, value):
        self._drawSticks = value

    """When an other finger is choosen, a calibration step is needed."""
    def changeFinger(self):
        self._nbUpdateColor = 0
        self._calibrationDone = False

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_DEPTH_TEST)

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        self.set_projection(self._near, self._far, self._fovy)
        self.updateGL()

    """Main function to draw the 3D world, sticks, mesh, triangles, grid, coord sys ..."""
    def paintGL(self):
        self._lock.acquire()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self._modelview_matrix)
        self._paintGLCoorsystem()
        self._paintGLGrid()
        if (self._calibrationDone == True):
            #rospy.loginfo('paintGL -> max values = %s', str(self._maxValues))
            #rospy.loginfo('paintGL -> _calibrationDone = %s', str(self._calibrationDone))
            self._calculateHeightValues()
            if (self._drawSticks):
                self._paintAllSticks()
            if (self._drawTriangles):
                self._paintTrianglesMesh()
            if (self._drawMesh):
                self._paintLinesMesh()
        self._lock.release()

    def _set_default_view(self):
        self.makeCurrent()
        self.reset_view()
        self.rotate((0, 0, 1), 45)
        self.rotate((1, 0, 0), -45)
        self.translate((0, -3, -15))

    """Set the X and Y positions of each electrodes"""
    def _set_electrode_positions(self):
        self._xPositions = [7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 4.38, 6.38, 6.38, 8.38, 7.58, 11.28, 14.78, 16.58, 19.08, 21.98, 11.38, 18.38, 22.18]
        self._yPositions = [6.45, 3.65, 3.65, 6.45, 3.65, 6.45, 0.00, 1.95, -1.95, 0.00, -6.45, -3.65, -3.65, -6.45, -3.65, -6.45, 0.00, 0.00, 0.00]

    """Paint colored triangles surface on the top of the sticks."""
    def _paintTrianglesMesh(self):
        glBegin(GL_TRIANGLES)
        for coords in self._coord_triangles:
            for ind in coords:
                x = self._xPositions[ind-1]
                y = self._yPositions[ind-1]
                z = self._height_values[ind-1] + 1
                glColor3f(self._red[ind-1], self._green[ind-1], self._blue[ind-1])
                glVertex3f(x,y,z)
        glEnd()

    """Paint lines representing mesh of triangles surface on the top of the sticks."""
    def _paintLinesMesh(self):
        glBegin(GL_LINES)
        for coords in self._coord_triangles:

            ind0 = coords[0]
            x0 = self._xPositions[ind0-1]
            y0 = self._yPositions[ind0-1]
            z0 = self._height_values[ind0-1] + 1

            ind1 = coords[1]
            x1 = self._xPositions[ind1-1]
            y1 = self._yPositions[ind1-1]
            z1 = self._height_values[ind1-1] + 1

            ind2 = coords[2]
            x2 = self._xPositions[ind2-1]
            y2 = self._yPositions[ind2-1]
            z2 = self._height_values[ind2-1] + 1

            glColor3f(255.0, 255.0, 255.0)
            glVertex3f(x0,y0,z0)
            glVertex3f(x1,y1,z1)
            glVertex3f(x1,y1,z1)
            glVertex3f(x2,y2,z2)
            glVertex3f(x2,y2,z2)
            glVertex3f(x1,y1,z1)

        glEnd()

    """Calculate the Height values of each stick"""
    def _calculateHeightValues(self):
        for i in range(self._num_electrodes):
            self._height_values[i] = float(10*self._values[i])/float(self._maxValues[i])

    """Paint all the 3D sticks in the world"""
    def _paintAllSticks(self):
        #rospy.loginfo('_paintAllSticks -> values = %s', str(self._values))
        for i in range(self._num_electrodes):
            self._paintGLStick(i, self._xPositions[i],self._yPositions[i],2,self._height_values[i])

    """Paint a 3D stick in the world. The center of the stick is width/2."""
    """On the plane XY. The plane Z is for the height"""
    def _paintGLStick(self, i, x, y, width, height):
        radius = float(width/2.0)
        #rospy.loginfo('_paintGLStick i = %s, height = %s', str(i), str(height))
        glBegin(GL_QUADS) # Start Drawing The Box

        glColor3f(self._red[i], self._green[i], self._blue[i])
        glVertex3f(x-radius, y+radius, 0.0) # Top Right Of The Quad (Bottom)
        glVertex3f(x+radius, y+radius, 0.0) # Top Left Of The Quad (Bottom)
        glVertex3f(x+radius, y-radius, 0.0) # Bottom Left Of The Quad (Bottom)
        glVertex3f(x-radius, y-radius, 0.0) # Bottom Right Of The Quad (Bottom)

        glColor3f(self._red[i], self._green[i], self._blue[i])
        glVertex3f(x-radius, y+radius, height) # Top Right Of The Quad (Front)
        glVertex3f(x-radius, y+radius, 0.0) # Top Left Of The Quad (Front)
        glVertex3f(x+radius, y+radius, 0.0) # Bottom Left Of The Quad (Front)
        glVertex3f(x+radius, y+radius, height) # Bottom Right Of The Quad (Front)

        glColor3f(self._red[i], self._green[i], self._blue[i])
        glVertex3f(x-radius, y-radius, height) # Bottom Left Of The Quad (Back)
        glVertex3f(x-radius, y-radius, 0.0) # Bottom Right Of The Quad (Back)
        glVertex3f(x+radius, y-radius, 0.0) # Top Right Of The Quad (Back)
        glVertex3f(x+radius, y-radius, height) # Top Left Of The Quad (Back)

        glColor3f(self._red[i], self._green[i], self._blue[i])
        glVertex3f(x-radius, y+radius, height) # Top Right Of The Quad (Left)
        glVertex3f(x-radius, y+radius, 0.0) # Top Left Of The Quad (Left)
        glVertex3f(x-radius, y-radius, 0.0) # Bottom Left Of The Quad (Left)
        glVertex3f(x-radius, y-radius, height) # Bottom Right Of The Quad (Left)

        glColor3f(self._red[i], self._green[i], self._blue[i])
        glVertex3f(x+radius, y+radius, height) # Top Right Of The Quad (Right)
        glVertex3f(x+radius, y+radius, 0.0) # Top Left Of The Quad (Right)
        glVertex3f(x+radius, y-radius, 0.0) # Bottom Left Of The Quad (Right)
        glVertex3f(x+radius, y-radius, height) # Bottom Right Of The Quad (Right)

        glVertex3f(x-radius, y+radius, height) # Top Right Of The Quad (Top)
        glVertex3f(x+radius, y+radius, height) # Top Left Of The Quad (Top)
        glVertex3f(x+radius, y-radius, height) # Bottom Left Of The Quad (Top)
        glVertex3f(x-radius, y-radius, height) # Bottom Right Of The Quad (Top)

        glEnd() # Done Drawing The Quad

    """Paint a simple 3D Box in the world"""
    def _paintGLBox(self):
        self._position = (2.0, 2.0, 2.0) # Set fixed translation for now
        glTranslatef(*self._position) # Translate Box
        matrix = quaternion_matrix(self._orientation) # convert quaternion to translation matrix
        glMultMatrixf(matrix) # Rotate Box

        glBegin(GL_QUADS) # Start Drawing The Box
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 1.0, -1.0) # Top Right Of The Quad (Top)
        glVertex3f(-1.0, 1.0, -1.0) # Top Left Of The Quad (Top)
        glVertex3f(-1.0, 1.0, 1.0) # Bottom Left Of The Quad (Top)
        glVertex3f(1.0, 1.0, 1.0) # Bottom Right Of The Quad (Top)

        glColor3f(0.5, 1.0, 0.5)
        glVertex3f(1.0, -1.0, 1.0) # Top Right Of The Quad (Bottom)
        glVertex3f(-1.0, -1.0, 1.0) # Top Left Of The Quad (Bottom)
        glVertex3f(-1.0, -1.0, -1.0) # Bottom Left Of The Quad (Bottom)
        glVertex3f(1.0, -1.0, -1.0) # Bottom Right Of The Quad (Bottom)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(1.0, 1.0, 1.0) # Top Right Of The Quad (Front)
        glVertex3f(-1.0, 1.0, 1.0) # Top Left Of The Quad (Front)
        glVertex3f(-1.0, -1.0, 1.0) # Bottom Left Of The Quad (Front)
        glVertex3f(1.0, -1.0, 1.0) # Bottom Right Of The Quad (Front)

        glColor3f(0.5, 0.5, 1.0)
        glVertex3f(1.0, -1.0, -1.0) # Bottom Left Of The Quad (Back)
        glVertex3f(-1.0, -1.0, -1.0) # Bottom Right Of The Quad (Back)
        glVertex3f(-1.0, 1.0, -1.0) # Top Right Of The Quad (Back)
        glVertex3f(1.0, 1.0, -1.0) # Top Left Of The Quad (Back)

        glColor3f(1.0, 0.5, 0.5)
        glVertex3f(-1.0, 1.0, 1.0) # Top Right Of The Quad (Left)
        glVertex3f(-1.0, 1.0, -1.0) # Top Left Of The Quad (Left)
        glVertex3f(-1.0, -1.0, -1.0) # Bottom Left Of The Quad (Left)
        glVertex3f(-1.0, -1.0, 1.0) # Bottom Right Of The Quad (Left)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 1.0, -1.0) # Top Right Of The Quad (Right)
        glVertex3f(1.0, 1.0, 1.0) # Top Left Of The Quad (Right)
        glVertex3f(1.0, -1.0, 1.0) # Bottom Left Of The Quad (Right)
        glVertex3f(1.0, -1.0, -1.0) # Bottom Right Of The Quad (Right)
        glEnd() # Done Drawing The Quad

    """Paint the 3D coord system of the world"""
    def _paintGLCoorsystem(self):
        glLineWidth(10.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(1.0, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 1.0, 0.0)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 1.0)
        glEnd()

    """Paint the 3D Grid of the world"""
    def _paintGLGrid(self):
        resolutionMillimeters = 1
        griddedAreaSize = 100
        glLineWidth(1.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(griddedAreaSize, 0, 0)
        glVertex3f(-griddedAreaSize, 0, 0)
        glVertex3f(0, griddedAreaSize, 0)
        glVertex3f(0, -griddedAreaSize, 0)
        numOfLines = int(griddedAreaSize / resolutionMillimeters)
        for i in range(numOfLines):
            glVertex3f(resolutionMillimeters * i, -griddedAreaSize, 0)
            glVertex3f(resolutionMillimeters * i, griddedAreaSize, 0)
            glVertex3f(griddedAreaSize, resolutionMillimeters * i, 0)
            glVertex3f(-griddedAreaSize, resolutionMillimeters * i, 0)
            glVertex3f(resolutionMillimeters * (-i), -griddedAreaSize, 0)
            glVertex3f(resolutionMillimeters * (-i), griddedAreaSize, 0)
            glVertex3f(griddedAreaSize, resolutionMillimeters * (-i), 0)
            glVertex3f(-griddedAreaSize, resolutionMillimeters * (-i), 0)
        glEnd()

    def get_view_matrix(self):
        return self._modelview_matrix.tolist()

    def set_view_matrix(self, matrix):
        self._modelview_matrix = numpy.array(matrix)

    def set_projection(self, near, far, fovy):
        self._near = near
        self._far = far
        self._fovy = fovy
        self.makeCurrent()
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        height = max(self.height(), 1)
        gluPerspective(self._fovy, float(self.width()) / float(height), self._near, self._far)
        self.updateGL()

    def reset_view(self):
        # scene pos and size
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self._modelview_matrix = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.view_all()

    def reset_rotation(self):
        self._modelview_matrix[0] = [1.0, 0.0, 0.0, 0.0]
        self._modelview_matrix[1] = [0.0, 1.0, 0.0, 0.0]
        self._modelview_matrix[2] = [0.0, 0.0, 1.0, 0.0]
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self._modelview_matrix)

    def translate(self, trans):
        # translate the object
        self.makeCurrent()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslated(trans[0], trans[1], trans[2])
        glMultMatrixd(self._modelview_matrix)
        # update _modelview_matrix
        self._modelview_matrix = glGetDoublev(GL_MODELVIEW_MATRIX)

    def rotate(self, axis, angle):
        # rotate the object
        self.makeCurrent()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        t = [self._modelview_matrix[3][0], self._modelview_matrix[3][1], self._modelview_matrix[3][2]]
        glTranslatef(t[0], t[1], t[2])
        glRotated(angle, axis[0], axis[1], axis[2])
        glTranslatef(-t[0], -t[1], -t[2])
        glMultMatrixd(self._modelview_matrix)
        # update _modelview_matrix
        self._modelview_matrix = glGetDoublev(GL_MODELVIEW_MATRIX)

    def view_all(self):
        self.translate([-self._modelview_matrix[0][3], -self._modelview_matrix[1][3], -self._modelview_matrix[2][3] - self._radius / 2.0])

    def wheelEvent(self, event):
        # only zoom when no mouse buttons are pressed, to prevent interference with other user interactions
        if event.buttons() == Qt.NoButton:
            d = float(event.angleDelta().y()) / 200.0 * self._radius
            self.translate([0.0, 0.0, d])
            self.updateGL()
            event.accept()

    def mousePressEvent(self, event):
        self._last_point_2d = event.pos()
        self._last_point_3d_ok, self._last_point_3d = self._map_to_sphere(self._last_point_2d)

    def mouseMoveEvent(self, event):
        new_point_2d = event.pos()
        if not self.rect().contains(new_point_2d):
            return
        new_point_3d_ok, new_point_3d = self._map_to_sphere(new_point_2d)
        dy = float(new_point_2d.y() - self._last_point_2d.y())
        h = float(self.height())

        # left button: rotate around center
        if event.buttons() == Qt.LeftButton and event.modifiers() == Qt.NoModifier:
            if self._last_point_3d_ok and new_point_3d_ok:
                cos_angle = numpy.dot(self._last_point_3d, new_point_3d)
                if abs(cos_angle) < 1.0:
                    axis = numpy.cross(self._last_point_3d, new_point_3d)
                    angle = 2.0 * math.acos(cos_angle) * 180.0 / math.pi
                    self.rotate(axis, angle)

        # middle button (or left + shift): move in x-y-direction
        elif event.buttons() == Qt.MidButton or (event.buttons() == Qt.LeftButton and event.modifiers() == Qt.ShiftModifier):
            dx = float(new_point_2d.x() - self._last_point_2d.x())
            w = float(self.width())
            z = -self._modelview_matrix[3][2] / self._modelview_matrix[3][3]
            n = 0.01 * self._radius
            up = math.tan(self._fovy / 2.0 * math.pi / 180.0) * n
            right = up * w / h
            self.translate([2.0 * dx / w * right / n * z, -2.0 * dy / h * up / n * z, 0.0])

        # left and middle button (or left + ctrl): move in z-direction
        elif event.buttons() == (Qt.LeftButton | Qt.MidButton) or (event.buttons() == Qt.LeftButton and event.modifiers() == Qt.ControlModifier):
            delta_z = self._radius * dy * 2.0 / h
            self.translate([0.0, 0.0, delta_z])

        # remember the new points and flag
        self._last_point_2d = new_point_2d
        self._last_point_3d = new_point_3d
        self._last_point_3d_ok = new_point_3d_ok

        # trigger redraw
        self.updateGL()

    def mouseReleaseEvent(self, _event):
        self._last_point_3d_ok = False

    def _map_to_sphere(self, pos):
        v = [0.0, 0.0, 0.0]
        # check if inside widget
        if self.rect().contains(pos):
            # map widget coordinates to the centered unit square [-0.5..0.5] x [-0.5..0.5]
            v[0] = float(pos.x() - 0.5 * self.width()) / self.width()
            v[1] = float(0.5 * self.height() - pos.y()) / self.height()
            # use Pythagoras to compute z (the sphere has radius sqrt(2.0*0.5*0.5))
            v[2] = math.sqrt(max(0.5 - v[0] * v[0] - v[1] * v[1], 0.0))
            # normalize direction to unit sphere
            v = numpy.array(v) / numpy.linalg.norm(v)
            return True, v
        else:
            return False, v

    """Calculate and store (_maxValues array) the maximum values of each electrodes."""
    def _calculateMaxValues(self, values):
        for i in range(len(values)):
            self._maxValues[i] = max(self._maxValues[i],values[i])
        #rospy.loginfo('maxvalues = %s !.', str(self._maxValues))

    """get the list of electrode values from topic callback method.
    Convert each value to rgb color"""
    def updateColor(self, value):
        #rospy.loginfo('updateColor !.')
        self._lock.acquire()
        self._nbUpdateColor +=1
        self._values = value

        for i in range( self._num_electrodes):
            self._red[i] = self.__color_array[self._max_reading-value[i],0]
            self._green[i] = self.__color_array[self._max_reading-value[i],1]
            self._blue[i] = self.__color_array[self._max_reading-value[i],2]

        if (self._nbUpdateColor < 10):
            self._calculateMaxValues(self._values)
        else:
            self._calibrationDone = True
        #rospy.loginfo('updateColor -> values = %s', str(self._values))
        self._lock.release()
