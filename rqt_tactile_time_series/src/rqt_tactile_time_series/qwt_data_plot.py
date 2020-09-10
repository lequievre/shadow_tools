#!/usr/bin/env python

# -*- coding: utf-8 -*-
from __future__ import division
import math
import sys
import rospy

from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt import QtCore

#from pyqtgraph.Qt.QtCore import QEvent, QSize, QPointF, Qt, Signal, Slot, qWarning
#from pyqtgraph.Qt.QtGui import QColor, QPen, QBrush, QVector2D

#import qwt 

import pyqtgraph as pg

from numpy import arange, zeros, concatenate

'''This class contains a widget to plot curves by using QWt library'''

class QwtDataPlot(pg.PlotWidget):

    def __init__(self, name = None):
        pg.PlotWidget.__init__(self)
        
        rospy.loginfo('QwtDataPlot -> __init__')

        # clear the widget
        self.clear()

        # set the title of the widget
        self.setTitle(name, color="b", size="20pt")
        
        # set X and Y axis title
        styles = {'color':'r', 'font-size':'20px'}
        self.setLabel('left', 'Value', **styles)
        self.setLabel('bottom', 'Time', **styles)

        # set the color of background
        # 'w' = white
        # or : self.setBackground((255, 255, 255))
        self.setBackground('w')

        # enable legend for curves
        self.addLegend()
        #self.insertLegend(Qwt.QwtLegend(), Qwt.QwtPlot.BottomLegend)
        
        #self.setXRange(0, 10, padding=0)
        #self.setYRange(20, 55, padding=0)

        # default with of curve
        self._curve_width = 3

        # enable grid on the widget
        self.showGrid(x=True, y=True)
        
        # dic to memory the curves items object
        self._curves = {}

    # enable to change the default curve width
    def update_curve_with(self, curve_id, curve_color, curve_width):
        self._curve_width = curve_width
        pen = QtGui.QPen(curve_color)
        pen.setWidth(curve_width)
        self._curves[curve_id].setPen(pen)

    # add a curve to the widget
    def add_curve(self, curve_id, curve_name, curve_color=QtGui.QColor(QtCore.Qt.blue), markers_on=False):
        rospy.loginfo('QwtDataPlot -> add_curve id =%s', str(curve_id))
        curve_id = str(curve_id)
        if curve_id in self._curves:
            return
        pen = QtGui.QPen(curve_color)
        pen.setWidth(self._curve_width)
        self._curves[curve_id] = self.plot([], [], name=curve_id, pen=pen)
        self._curves[curve_id].setPen(pen)
        
    # remove a curve to the widget
    def remove_curve(self, curve_id):
        rospy.loginfo('QwtDataPlot -> remove_curve id =%s', str(curve_id))
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self.removeItem(self._curves[curve_id])
            del self._curves[curve_id]

    # set values to plot on a curve
    def set_values(self, curve_id, data_x, data_y):
        rospy.loginfo('QwtDataPlot -> set_values')
        self._curves[curve_id].setData(data_x, data_y)

    # force to redraw the widget
    def redraw(self):
        self.replot()
