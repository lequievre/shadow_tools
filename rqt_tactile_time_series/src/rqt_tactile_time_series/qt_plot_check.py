#!/usr/bin/env python

import rospy

from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt import QtCore

#from pyqtgraph.Qt.QtGui import QColor
#from pyqtgraph.Qt.QtGui import QWidget, QSizePolicy, QVBoxLayout, QHBoxLayout
#from pyqtgraph.Qt.QtGui import QCheckBox, QComboBox
#from pyqtgraph.Qt.QtCore import Qt, QObject, pyqtSlot, Signal, Slot

#from pyqtgraph.Qt.QtCore import QTimer

from qwt_data_plot import QwtDataPlot
import matplotlib.pylab as plt

import threading

'''This class contains a widget to plot curves and checked boxes to add or remove curves'''

class QtPlotChecked(QtGui.QWidget):
    def __init__(self, name=None, parent=None):
        super(QtPlotChecked, self).__init__(parent)
        
        rospy.loginfo('QtPlotChecked -> __init__')

        # name of the plot widget
        self._name = name

        # names of the check box. True = checked, False = unchecked
        self._listNamesValuesClicked = { 'pac0':True, 'pac1':False, 'pdc':False, 'tac':False, 'tdc':False }

        # colors of the curve associate to a check box
        self._listColorValues = { 'pac0':QtGui.QColor(QtCore.Qt.red), 'pac1':QtGui.QColor(QtCore.Qt.blue), 'pdc':QtGui.QColor(QtCore.Qt.darkMagenta), 'tac':QtGui.QColor(QtCore.Qt.black), 'tdc':QtGui.QColor(QtCore.Qt.darkGray) }

        # plot's values of the curves. The id and name of the curve = the name of the check box
        # y values of the curve
        self._dicValues = {}
        for v in self._listNamesValuesClicked:
            self._dicValues[v] = []

        # values of the time (x axis of the curve)
        self._listTimes = []

        # boolean to detect the first setting of values
        self._valuesHasChanged = False

        # max values to plot
        self._maxValues = 100

        # necessary to calculate the time duration
        self._firstTime = float(0)

        # The QWt Plot widget
        self._qwtDataPlot = QwtDataPlot(self._name)

         # A combo box to choose the width of the curve
        self._combo = QtGui.QComboBox()

        # different with for the curves
        combolist = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
        self._combo.addItems(combolist)
        self._combo.setCurrentIndex(2)

        # connect to the slot '_combo_chosen' when the width's curve has changed
        #QtCore.QObject.connect(self._combo,SIGNAL("activated(QString)"),self,SLOT("_combo_chosen(QString)"))
        self._combo.activated.connect(self._combo_chosen)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(self._qwtDataPlot)

        vboxCheck = QtGui.QVBoxLayout()

        # for each check box checked, add a curve to the QWt plot widget
        self._listCheck = {}
        for v in self._listNamesValuesClicked:
            self._listCheck[v]= QtGui.QCheckBox(v)
            if (self._listNamesValuesClicked[v] == True):
                self._qwtDataPlot.add_curve(v, v, self._listColorValues[v])
                self._listCheck[v].setCheckState(QtCore.Qt.Checked)
            vboxCheck.addWidget(self._listCheck[v])

        vboxCheck.addWidget(self._combo)

        # connect to a slot, when the check box is checked
        self._listCheck['pac0'].stateChanged.connect(self._checkedSlot_pac0)
        self._listCheck['pac1'].stateChanged.connect(self._checkedSlot_pac1)
        self._listCheck['pdc'].stateChanged.connect(self._checkedSlot_pdc)
        self._listCheck['tac'].stateChanged.connect(self._checkedSlot_tac)
        self._listCheck['tdc'].stateChanged.connect(self._checkedSlot_tdc) 
        
        #QtCore.QObject.connect(self._listCheck['pac0'],SIGNAL("stateChanged(int)"),self,SLOT("_checkedSlot_pac0(int)"))
        #QtCore.QObject.connect(self._listCheck['pac1'],SIGNAL("stateChanged(int)"),self,SLOT("_checkedSlot_pac1(int)"))
        #QtCore.QObject.connect(self._listCheck['pdc'],SIGNAL("stateChanged(int)"),self,SLOT("_checkedSlot_pdc(int)"))
        #QtCore.QObject.connect(self._listCheck['tac'],SIGNAL("stateChanged(int)"),self,SLOT("_checkedSlot_tac(int)"))
        #QtCore.QObject.connect(self._listCheck['tdc'],SIGNAL("stateChanged(int)"),self,SLOT("_checkedSlot_tdc(int)"))

        hbox = QtGui.QHBoxLayout()
        hbox.addLayout(vbox)
        hbox.addLayout(vboxCheck)

        self.setLayout(hbox)

        # a thread lock is necessary to 'mutex' the values and time data between the timer that update the graphic,
        # and the 'addValues' method called for each topic callback
        self._lock = threading.Lock()

        # timer to update the graphic each 100 ms
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._update_figure)
        self._timer.start(100)

    @QtCore.pyqtSlot(int)
    def _combo_chosen(self, value):
        rospy.loginfo('_combo_chosen = %s !',value)
        for v in self._listNamesValuesClicked:
            if (self._listNamesValuesClicked[v] == True):
                self._qwtDataPlot.update_curve_with(v, self._listColorValues[v], int(self._combo.itemData(value)))

    @QtCore.pyqtSlot(int)
    def _checkedSlot_pac0(self,value):
        rospy.loginfo('_checkedSlot_pac0 !')
        name = 'pac0'
        if (value == QtCore.Qt.Checked):
            self._listNamesValuesClicked[name] = True
            self._qwtDataPlot.add_curve(name, name, self._listColorValues[name])
        else:
            self._listNamesValuesClicked[name] = False
            self._qwtDataPlot.remove_curve(name)

    @QtCore.pyqtSlot(int)
    def _checkedSlot_pac1(self,value):
        rospy.loginfo('_checkedSlot_pac1 !')
        name = 'pac1'
        if (value == QtCore.Qt.Checked):
            self._listNamesValuesClicked[name] = True
            self._qwtDataPlot.add_curve(name, name, self._listColorValues[name])
        else:
            self._listNamesValuesClicked[name] = False
            self._qwtDataPlot.remove_curve(name)

    @QtCore.pyqtSlot(int)
    def _checkedSlot_pdc(self,value):
        rospy.loginfo('_checkedSlot_pdc !')
        name = 'pdc'
        if (value == QtCore.Qt.Checked):
            self._listNamesValuesClicked[name] = True
            self._qwtDataPlot.add_curve(name, name, self._listColorValues[name])
        else:
            self._listNamesValuesClicked[name] = False
            self._qwtDataPlot.remove_curve(name)

    @QtCore.pyqtSlot(int)
    def _checkedSlot_tac(self,value):
        rospy.loginfo('_checkedSlot_tac !')
        name = 'tac'
        if (value == QtCore.Qt.Checked):
            self._listNamesValuesClicked[name] = True
            self._qwtDataPlot.add_curve(name, name, self._listColorValues[name])
        else:
            self._listNamesValuesClicked[name] = False
            self._qwtDataPlot.remove_curve(name)

    @QtCore.pyqtSlot(int)
    def _checkedSlot_tdc(self,value):
        rospy.loginfo('_checkedSlot_tdc !')
        name = 'tdc'
        if (value == QtCore.Qt.Checked):
            self._listNamesValuesClicked[name] = True
            self._qwtDataPlot.add_curve(name, name, self._listColorValues[name])
        else:
            self._listNamesValuesClicked[name] = False
            self._qwtDataPlot.remove_curve(name)

    # timer update graphic method
    def _update_figure(self):
        if (self._firstTime != 0 and self._valuesHasChanged == True):
            self._lock.acquire()
            try:
                for v in self._listNamesValuesClicked:
                    if (self._listNamesValuesClicked[v]==True):
                       self._qwtDataPlot.set_values(v, self._listTimes, self._dicValues[v])
                self._qwtDataPlot.redraw()
            finally:
                self._lock.release()

    # at each topic callback, it is necessary to add values
    def addValues(self, headData, tactileData):
        pac0 = tactileData.pac0
        pac1 = tactileData.pac1
        pdc = tactileData.pdc
        tac = tactileData.tac
        tdc = tactileData.tdc

        time = headData.stamp.secs+(headData.stamp.nsecs/1e9)

        if (self._firstTime == 0):
             self._firstTime = time

        timeDuration = time - self._firstTime

        self._lock.acquire()
        try:
            # if it exceeds the maximum, the first element is removed before adding the new value
            if (len(self._listTimes) >= self._maxValues):
                del self._listTimes[0]
                for v in self._listNamesValuesClicked:
                    del self._dicValues[v][0]

            self._listTimes.append(timeDuration)
            self._dicValues['pac0'].append(pac0)
            self._dicValues['pac1'].append(pac1)
            self._dicValues['pdc'].append(pdc)
            self._dicValues['tac'].append(tac)
            self._dicValues['tdc'].append(tdc)
            self._valuesHasChanged = True
        finally:
            self._lock.release()
