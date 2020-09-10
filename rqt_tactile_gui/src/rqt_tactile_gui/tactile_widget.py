from python_qt_binding.QtWidgets import QWidget, QSizePolicy
from python_qt_binding.QtGui import QPainter, QPen, QColor, QFont, QTransform
from python_qt_binding.QtCore import QRectF, Qt
import numpy
import math
import rospy
import matplotlib
import matplotlib.pyplot
import threading

'''This class contains the graphical widget that drawn electrodes of a finger. The parameter draw_values_only is set when you want to draw only the electrodes values'''

class TactileWidget(QWidget):

    def __init__(self, text_info = '', draw_values_only = False, width=200, height=200):
        """Call the constructor of QWidget"""
        super(TactileWidget, self).__init__()

        """Set the number of electrodes"""
        self._num_electrodes = 19

        self._text_info = text_info

        self._draw_values_only = draw_values_only
        self._values = ['0'] *  self._num_electrodes

        self._bioTac_width = 45
        self._bioTac_height = 45

        """Set the max value of electrodes model file"""
        self._max_reading = 4095

        self._sensing_electrode = []

        for i in range( self._num_electrodes):
            self._sensing_electrode.append(QRectF())

        self._set_sensing_electrode()

        """Set the size of the Widget"""
        self.resize(width,height)
        self.setMinimumSize(self.size())

        #self.setFixedSize(width,height)
        #self.setMaximumSize(self.size())
        #self.setSizePolicy( QSizePolicy( QSizePolicy.Fixed, QSizePolicy.Fixed, True ))

        """Fill the background to White"""
        self.setStyleSheet("background-color:white;")
        self.setAutoFillBackground(True)

        """Set a color array (for the 4096 values) by using a color map.
        The color array list for each value (0 to 4096) contains a sub list [r,g,b,a].
        """
        self.__color_map = matplotlib.cm.ScalarMappable(cmap='jet')
        self.__color_array = numpy.linspace(0,1,self._max_reading+1)
        self.__color_array = self.__color_map.to_rgba(self.__color_array)

        '''a threading lock is enable to concurrent data access '''
        self._lock = threading.Lock()

        """Set color List of electrodes"""
        self._red = [255] *  self._num_electrodes
        self._green = [255] *  self._num_electrodes
        self._blue = [255] *  self._num_electrodes

        rospy.loginfo('Create tactile widget (%s,%s).', width, height)

        """Update the Widget to draw it with paintEvent method"""
        self.update()

    '''Define the initial positions of electrodes with a QRectf'''
    def _set_sensing_electrode(self):
        self._sensing_electrode[0].setRect(7.58, 6.45, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[1].setRect(11.28, 3.65, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[2].setRect(14.78, 3.65, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[3].setRect(16.58, 6.45, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[4].setRect(19.08, 3.65, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[5].setRect(21.98, 6.45, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[6].setRect(4.38, 0.00, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[7].setRect(6.38, 1.95, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[8].setRect(6.38, -1.95, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[9].setRect(8.38, 0.00, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[10].setRect(7.58, -6.45, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[11].setRect(11.28, -3.65, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[12].setRect(14.78, -3.65, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[13].setRect(16.58, -6.45, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[14].setRect(19.08, -3.65, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[15].setRect(21.98, -6.45, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[16].setRect(11.38, 0.00, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[17].setRect(18.38, 0.00, self._bioTac_width , self._bioTac_height)
        self._sensing_electrode[18].setRect(22.18, 0.00, self._bioTac_width , self._bioTac_height)


    """This function is obsolete"""
    def __convertRawToRgb(self, value):
        ''' RGB default values (i.e. blue at the beginning)'''
        red = 0
        green = 0
        blue = 255

        '''thresholds to convert electrode impedance to RGB values'''
        e0 = 0
        e1 = 1000
        e2 = 2000
        e3 = 3000
        e4 = 4095

        if (value >= e0 and value < e1):
            red = 255
            green = 255*((value-e0)/(e1-e0))
            blue = 0
        elif (value >= e1 and value < e2):
            red = 255*((e2-value)/(e2-e1))
            green = 255
            blue = 0
        elif (value >= e2 and value < e3):
            red = 0
            green = 255
            blue = 255*((value-e2)/(e3-e2))
        elif (value >= e3 and value <= e4):
            red = 0
            green = 255*((e4-value)/(e4-e3))
            blue = 255
        return (red, green, blue)

    """get the list of electrode values from topic callback method.
    Convert each value to rgb color"""
    def updateColor(self,value):
        #rospy.loginfo('updateColor !.')
        self._lock.acquire()
        self._values = value
        for i in range( self._num_electrodes):
            #rgb = self.__convertRawToRgb(value[i])
            #red = rgb[0]
            #green = rgb[1]
            #blue = rgb[2]
            red = self.__color_array[self._max_reading-value[i],0]*255
            green = self.__color_array[self._max_reading-value[i],1]*255
            blue = self.__color_array[self._max_reading-value[i],2]*255
            self._red[i] = red
            self._green[i] = green
            self._blue[i] = blue
            #rospy.loginfo('i = %s, color= %s', str(i), str(self.__list_ElectrodeColors[i]))
        self._lock.release()
        self.update()

    """Draw the graphical widget"""
    def paintEvent(self, x):
            self._lock.acquire()

            painter = QPainter(self)

            # Draw a rectangle around the widget
            painter.drawRect(0,0,self.width()-5,self.height()-5)

            tmp_x, tmp_y, tmp_w, tmp_h = 0.0, 0.0, 0.0, 0.0
            factor = 17.5
            offset_x1 = 150.0
            offset_y1 = -50.0
            offset_x2 = offset_x1 + 12.5
            offset_y2 = offset_y1 + 4.0
            offset_x3 = offset_x1 + 4.5
            offset_y3 = offset_y1 + 4.0
            offset_x4 = offset_x1 + 3.5
            offset_y4 = offset_y1 + 4.0

            font_size_1 = 24
            font_size_2 = 12

            # Initial the electrode positions
            self._set_sensing_electrode()

            painter.setRenderHint(QPainter.Antialiasing, True)
            painter.setPen(QPen(Qt.black, 1, Qt.SolidLine, Qt.RoundCap))

            for i in range(len(self._sensing_electrode)):

                # Get the left corner, width and height of rect electrode
                tmp_x = self._sensing_electrode[i].left()
                tmp_y = self._sensing_electrode[i].top()
                tmp_w = self._sensing_electrode[i].width()
                tmp_h = self._sensing_electrode[i].height()

                # Adjust position of rect electrode
                self._sensing_electrode[i].setRect(tmp_y*factor + offset_x1, tmp_x*factor + offset_y1, tmp_w, tmp_h)
                #rospy.loginfo('i = %s, x = %s, y = %s, w = %s, h = %s ', str(i), str(self._sensing_electrode[i].left()), str(self._sensing_electrode[i].top()), str(self._sensing_electrode[i].width()), str(self._sensing_electrode[i].height()))

                if (self._draw_values_only == True):
                    # Draw the value, if the widget is a "value widget" only
                    painter.setFont(QFont("Arial", font_size_2))
                    painter.drawText(self._sensing_electrode[i], str(self._values[i]))
                else:
                    # Draw ellipse representing electrode into a rect
                    painter.setBrush(QColor(self._red[i], self._green[i], self._blue[i]))
                    painter.drawEllipse(self._sensing_electrode[i])
                    painter.setFont(QFont("Arial", font_size_1))

                    # Adjust x,y position for the electrode numbers
                    if (i<9):
                        self._sensing_electrode[i].setRect(tmp_y*factor + offset_x2, tmp_x*factor + offset_y2, tmp_w, tmp_h)
                    else:
                        self._sensing_electrode[i].setRect(tmp_y*factor + offset_x3, tmp_x*factor + offset_y3, tmp_w, tmp_h)

                    painter.drawText(self._sensing_electrode[i], str(i+1))

            self._lock.release()

            # Draw the title of the widget (the name of the finger)
            x = 5
            y = 5
            painter.setFont(QFont("Arial", font_size_1))
            painter.save()
            painter.translate(x, y)
            painter.rotate(90)
            # Draw the name of the finger
            painter.drawText(0, 0, self._text_info)
            painter.restore()
