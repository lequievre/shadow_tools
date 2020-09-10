import os
import rospy
import rospkg
from sr_robot_msgs.msg import BiotacAll

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QSizePolicy
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QHBoxLayout, QGridLayout, QPushButton, QLabel, QFrame
from python_qt_binding.QtCore import QObject, pyqtSignal

from tactile_widget import TactileWidget

'''This class define a Qt signal that contains electrodes values list of each finger'''
class TactileValuesSignal(QObject):
    signal = pyqtSignal(list)

'''This class contains a rqt plugin to draw colors and values of 19 tactile electrodes of each fingers'''

class TactilePlugin(Plugin):

    def __init__(self, context):
        super(TactilePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TactilePlugin')

        # create a Qt signal. This signal is emited at each 'callback values' on 'tactile' topic
        self.__tactileValuesSignal = TactileValuesSignal()

        # set the number of fingers
        self.__nbFingers = 5

        # set the text to display for each graphical widget finger
        self.__nameFingers_list = ['FF','MF','RF','LF','Thumb']

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

        self.setupUI()

        if context.serial_number() > 1:
            self.__tactileWidgets.setWindowTitle(self.__tactileWidgets.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self.__tactileWidgets)

        # connect the signal Qt to an handler method
        # The handler method update the electrodes values of the graphical widget
        self.__tactileValuesSignal.signal.connect(self.__tactileValuesSignalHandler)

        # subscribe to the 'tactile' topic
        self.__subscribeToTopic()

    def setupUI(self):
        # Create QWidget
        self.__tactileWidgets = QWidget()
        self.__tactileWidgets.setMinimumSize(800,400)
        #self._tactileWidgets.setSizePolicy( QSizePolicy( QSizePolicy.Fixed, QSizePolicy.Fixed, True ));

        self.__tactileWidgets.setStyleSheet("background-color:white;")
        self.__tactileWidgets.setAutoFillBackground(True)

        self.__gridTactileWidgets = QGridLayout()
        self.__tactileWidgets.setLayout(self.__gridTactileWidgets)

        # the 'tactile widget' list contains color graphical widget (1 per finger)
        # the 'tactile values widget' list contains only values graphical widget (1 per finger)
        self.__tactileWidget_list = []
        self.__tactileValuesWidget_list = []

        for i in range(self.__nbFingers):
            self.__tactileWidget_list.append(TactileWidget(self.__nameFingers_list[i],150,150))
            self.__tactileValuesWidget_list.append(TactileWidget(self.__nameFingers_list[i],True,150,150))

            self.__gridTactileWidgets.addWidget(self.__tactileWidget_list[i],2,i)
            self.__gridTactileWidgets.addWidget(self.__tactileValuesWidget_list[i],3,i)

        # --------------------------------------
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self.__tactileWidgets.setWindowTitle('Shadow Tactile Sensor Viewer')

    # update the electrode values for each graphical widget finger
    def __tactileValuesSignalHandler(self, value):
        #rospy.loginfo('tactileValuesSignalHandler !.')
        for i in range(self.__nbFingers):
            self.__tactileWidget_list[i].updateColor(value[i])
            self.__tactileValuesWidget_list[i].updateColor(value[i])

    # subscribe to the 'tactile' topic
    def __subscribeToTopic(self, topic_name='rh/tactile'):
        # BiotacAll is the name of the message Class. The name of the topic is by default 'tactile'
        self.__topicTactile = rospy.Subscriber(topic_name, BiotacAll, self.__readData_callback)
        rospy.loginfo('Subscribe to topic <%s>.' % topic_name)

    # 'callback' method to treat a topic's message
    def __readData_callback(self,data):
        readings=[]
        for i in range(len(data.tactiles)):
            readings.append(data.tactiles[i].electrodes)
        #rospy.loginfo('Reading from topic <%s>.' % readings)
        self.__tactileValuesSignal.signal.emit(readings)

    # when the plugin is closed, think to unregister the topic
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
