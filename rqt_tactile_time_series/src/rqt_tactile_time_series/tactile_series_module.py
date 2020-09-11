#!/usr/bin/env python

import os
import rospy
import rospkg
from sr_robot_msgs.msg import BiotacAll

from qt_gui.plugin import Plugin

from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt import QtCore


#from pyqtgraph.Qt.QtGui import QWidget, QSizePolicy, QHBoxLayout, QGridLayout
#from pyqtgraph.Qt.QtGui import QFont
#from pyqtgraph.Qt.QtGui import QPushButton, QLabel, QFrame, QTabWidget
#from pyqtgraph.Qt.QtCore import QObject, QTimer, pyqtSignal

from qt_plot_check import QtPlotChecked

import random
import datetime
import threading

'''This class contains a rqt plugin to draw curves of different parameters of each fingers'''

class TactileSeriesPlugin(Plugin):

    def __init__(self,context):
        super(TactileSeriesPlugin, self).__init__(context)
        
        #rospy.loginfo('TactileSeriesPlugin -> __init__')

         # Give QObjects reasonable names
        self.setObjectName('TactilePlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # set the number of fingers
        self._nbFingers = 5

        # set the order and the name of each finger
        self._nameFingers_list = ['FF','MF','RF','LF','Thumb']

        # list of all widget that draw curve's widget and checked box (to choose parameters to draw)
        self._listCheckedDataPlot = []
        for i in range(self._nbFingers):
            self._listCheckedDataPlot.append(QtPlotChecked(self._nameFingers_list[i]))

        # draw User Interface
        self.setupUI()

        if context.serial_number() > 1:
            self.__tactileWidgets.setWindowTitle(self.__tactileWidgets.windowTitle() + (' (%d)' % context.serial_number()))

        # Add the main layout to the user interface
        context.add_widget(self.__tactileWidgets)

        # subscribe to Ros topic
        self.__subscribeToTopic()

        # define a mutex for concurrent access data
        self.__lock = threading.Lock()

    # define the UI
    def setupUI(self):
        self.__tactileWidgets = QtGui.QWidget()
        #self.__tactileWidgets.setMinimumSize(800,400)
        #self.__tactileWidgets.setStyleSheet("background-color:white;")
        #self.__tactileWidgets.setAutoFillBackground(True)

        self.__gridTactileWidgets = QtGui.QGridLayout()
        self.__tactileWidgets.setLayout(self.__gridTactileWidgets)

        self.__gridTactileWidgets.addWidget(self._listCheckedDataPlot[0],1,1)
        self.__gridTactileWidgets.addWidget(self._listCheckedDataPlot[1],1,2)
        self.__gridTactileWidgets.addWidget(self._listCheckedDataPlot[2],2,1)
        self.__gridTactileWidgets.addWidget(self._listCheckedDataPlot[3],2,2)
        self.__gridTactileWidgets.addWidget(self._listCheckedDataPlot[4],3,1)

        # --------------------------------------
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self.__tactileWidgets.setWindowTitle('Shadow Tactile Qwt Sensor Series Viewer')

    # subscribe to Ros topic 'tactile'
    def __subscribeToTopic(self, topic_name='rh/tactile'):
        # BiotacAll is the name of the message Class. The name of the topic is by default 'tactile'
        self.__topicTactile = rospy.Subscriber(topic_name, BiotacAll, self.__readData_callback)
        rospy.loginfo('Subscribe to topic <%s>.' % topic_name)

    # callback method for each message receive by the topic 'tactile'
    def __readData_callback(self,data):
        for i in range(self._nbFingers):
            self._listCheckedDataPlot[i].addValues(data.header,data.tactiles[i])

    # unregister topic before to shutdown the plugin
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.__topicTactile.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
