import os
import rospy
import rospkg
from sr_robot_msgs.msg import BiotacAll

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QSizePolicy
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel, QFrame, QComboBox, QGroupBox, QRadioButton, QTabWidget, QCheckBox
from python_qt_binding.QtCore import QObject, pyqtSignal, Qt

from tactile_widget import TactileWidget

'''This class define a Qt signal that contains electrodes values list of each finger'''
class TactileValuesSignal(QObject):
    signal = pyqtSignal(list)

class TactilePlugin(Plugin):

    def __init__(self, context):
        super(TactilePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TactilePlugin')

         # create a Qt signal. This signal is emited at each 'callback values' on 'tactile' topic
        self.__tactileValuesSignal = TactileValuesSignal()

        # set the number of fingers
        self.__nbFingers = 5

        self._indexFinger = 0

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

        self.__tactileWidgets.setStyleSheet("background-color:white;")
        self.__tactileWidgets.setAutoFillBackground(True)

        self.__mainLayout = QHBoxLayout()
        self.__tactileWidgets.setLayout(self.__mainLayout)

        self.__tactileWidget = TactileWidget(self.__nameFingers_list[0])
        sp_tactile = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sp_tactile.setHorizontalStretch(90)
        self.__tactileWidget.setSizePolicy(sp_tactile)

        self.__mainLayout.addWidget(self.__tactileWidget)

        self._tabs = QTabWidget()

        self._groupBoxFinger = QGroupBox("Active Finger")
        radio_ff = QRadioButton("FF")
        radio_ff.toggled.connect(self._radioFF_clicked)
        radio_mf = QRadioButton("MF")
        radio_mf.toggled.connect(self._radioMF_clicked)
        radio_rf = QRadioButton("RF")
        radio_rf.toggled.connect(self._radioRF_clicked)
        radio_lf = QRadioButton("LF")
        radio_lf.toggled.connect(self._radioLF_clicked)
        radio_thumb = QRadioButton("Thumb")
        radio_thumb.toggled.connect(self._radioTHUMB_clicked)
        radio_ff.setChecked(True)
        vbox_finger = QVBoxLayout()
        vbox_finger.addWidget(radio_ff)
        vbox_finger.addWidget(radio_mf)
        vbox_finger.addWidget(radio_rf)
        vbox_finger.addWidget(radio_lf)
        vbox_finger.addWidget(radio_thumb)
        vbox_finger.addStretch(1)
        self._groupBoxFinger.setLayout(vbox_finger)

        self._groupBoxDrawOptions = QGroupBox("Draw Options")
        vbox_draw_options = QVBoxLayout()
        self._cb_draw_stick = QCheckBox("Draw Sticks")
        self._cb_draw_stick.setCheckState(Qt.Checked)
        self._cb_draw_stick.stateChanged.connect(self._cb_draw_sticks_checked)
        self._cb_draw_mesh = QCheckBox("Draw Mesh")
        self._cb_draw_mesh.setCheckState(Qt.Unchecked)
        self._cb_draw_mesh.stateChanged.connect(self._cb_draw_mesh_checked)
        self._cb_draw_triangle = QCheckBox("Draw Triangles")
        self._cb_draw_triangle.setCheckState(Qt.Unchecked)
        self._cb_draw_triangle.stateChanged.connect(self._cb_draw_triangles_checked)
        vbox_draw_options.addWidget(self._cb_draw_stick)
        vbox_draw_options.addWidget(self._cb_draw_mesh)
        vbox_draw_options.addWidget(self._cb_draw_triangle)
        vbox_draw_options.addStretch(1)
        self._groupBoxDrawOptions.setLayout(vbox_draw_options)

        self._tabs.addTab(self._groupBoxFinger,"Choose a finger")
        self._tabs.addTab(self._groupBoxDrawOptions,"Options")

        self.__mainLayout.addWidget(self._tabs)

        # --------------------------------------
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self.__tactileWidgets.setWindowTitle('Shadow Tactile Sensor Viewer')


    def _cb_draw_sticks_checked(self, state):
        if (state == Qt.Checked):
            self.__tactileWidget.setDrawStricks(True)
        else:
            self.__tactileWidget.setDrawStricks(False)

    def _cb_draw_mesh_checked(self, state):
        if (state == Qt.Checked):
            self.__tactileWidget.setDrawMesh(True)
        else:
            self.__tactileWidget.setDrawMesh(False)

    def _cb_draw_triangles_checked(self, state):
        if (state == Qt.Checked):
            self.__tactileWidget.setDrawTriangles(True)
        else:
            self.__tactileWidget.setDrawTriangles(False)

    def _radioFF_clicked(self, enabled):
        if (enabled):
            self.__tactileWidget.changeFinger()
            self._indexFinger = 0

    def _radioMF_clicked(self, enabled):
        if (enabled):
            self.__tactileWidget.changeFinger()
            self._indexFinger = 1

    def _radioRF_clicked(self, enabled):
        if (enabled):
            self.__tactileWidget.changeFinger()
            self._indexFinger = 2

    def _radioLF_clicked(self, enabled):
        if (enabled):
            self.__tactileWidget.changeFinger()
            self._indexFinger = 3

    def _radioTHUMB_clicked(self, enabled):
        if (enabled):
            self.__tactileWidget.changeFinger()
            self._indexFinger = 4

    # update the electrode values for each graphical widget finger
    def __tactileValuesSignalHandler(self, value):
        #rospy.loginfo('tactileValuesSignalHandler !.')
        #for i in range(len(self.__tactileWidget_list)):
            #rospy.loginfo('i = %s, l = %s!.', str(i), str(value[i]))
         #   self.__tactileWidget_list[i].updateColor(value[i])
        self.__tactileWidget.updateColor(value[self._indexFinger])

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
