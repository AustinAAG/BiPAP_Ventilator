import numpy

#Standalone .py for display/edits
import matplotlib
matplotlib.use('Qt5Agg')

from PySide2.QtCore import QDateTime, Qt, QTimer
from PySide2.QtWidgets import (QMainWindow, QApplication, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QDoubleSpinBox, QVBoxLayout, QWidget)

import sys
import os
import random
from functools import partial

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

# supress deprecation warning
#import warnings
#warnings.filterwarnings("ignore", category=UserWarning)

#data logging
import csv;
import time;
from datetime import datetime;

#serial data
import serial
import serial.tools.list_ports

#generate file name with current date and time
path = "data"
if not os.path.exists(path):
    os.makedirs(path)
    
now = datetime.now()
date_time = now.strftime("%m")+"."+now.strftime("%d")+"."+now.strftime("%y")+"_("+now.strftime("%H_%M_%S")
filename = "DATA_" + date_time + ").csv";

#universal vraible for auto-scaling
scale = False

#create file and set headings
with open(os.path.join(path, filename), 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Flow rate", "Inhale", "Exhale", "Timestamp"])

#print available com ports
comPorts = list(serial.tools.list_ports.comports())
for device in comPorts:
    print(device)


#Ask for user input, loop until valid input is found
valid_input = False
port = input("Serial port: ")
deviceName = "";

while not valid_input:
    try:
        ser = serial.Serial(port, timeout = 3, baudrate = 115200)
        valid_input = True
        deviceName = port;
        print("Device found!")
    except:
        print("No device found!")
        valid_input = False
        port = input("Re-enter serial port: ")        


#FOR MATPLOT OBJECTS
class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(1, 1, 1)
        super(MplCanvas, self).__init__(fig)

#ACTUAL GUI CLASS
class RespiratorDisplay(QDialog):
    def __init__(self, *args, **kwargs):
        super(RespiratorDisplay, self).__init__(*args, **kwargs)

        #All the general layout things will go here, in init

        #Group with all the parameter functions
        self.makeParamBox()
        
        #Windows for plots
        self.makePlotBox()

        # Add Widgets together in final layout
        topLayout = QHBoxLayout()
        textBox = QLabel("Port: " + deviceName)
        topLayout.addStretch(1)
        topLayout.addWidget(textBox)

        bottomGroup = QGroupBox("Board")
        bottomLayout = QGridLayout()
        reset = QPushButton("Reset")
        reset.pressed.connect(partial(self.sendData,"R"))
        bottomLayout.addWidget(reset, 1, 1)
        bottomGroup.setLayout(bottomLayout)

        #show flowrate values
        self.makeUpdateBox()

        mainLayout = QGridLayout()
        mainLayout.addLayout(topLayout, 0, 0, 1, 2)
        mainLayout.addWidget(self.PlotBox, 1, 0, 2, 1)
        mainLayout.addWidget(self.ParamBox, 1, 1, 1, 1)
        mainLayout.addWidget(self.AlarmBox, 2, 1, 1, 1)
        #mainLayout.addWidget(self.ScaleBox, 3, 1, 1, 1) #Box currently isnt working
        mainLayout.addWidget(self.initBox, 4, 1, 1, 1)
        mainLayout.addWidget(bottomGroup, 4, 0, 1, 1)
        mainLayout.addWidget(self.UpdateBox, 3, 0, 1, 1)

        self.setLayout(mainLayout)
        self.setWindowTitle("Board Info")

    def makeUpdateBox(self):
        
        self.UpdateBox = QGroupBox("Updates")

        self.volumeInBox = QLabel("Tidal Inhale Volume (ml): ")
        self.volumeOutBox = QLabel("Tidal Exhale Volume (ml): ")
        
        updateLayout = QGridLayout()

        updateLayout.addWidget(self.volumeInBox)
        updateLayout.addWidget(self.volumeOutBox)
        
        self.UpdateBox.setLayout(updateLayout)        

    def makeParamBox(self):

        self.ParamBox = QGroupBox("Inputs")

        Option02 = QPushButton("%02")
        Option02.pressed.connect(partial(self.setpar,1))
        self.param02 = QDoubleSpinBox()
        #set range with this method
        self.param02.setMinimum(0.0)
        self.param02.setMaximum(1.0)
        self.param02.setDecimals(2)
        self.param02.setValue(0.21)

        OptionC02 = QPushButton("%C02")
        OptionC02.pressed.connect(partial(self.setpar,2))
        self.paramC02 = QDoubleSpinBox()
        self.paramC02.setMinimum(0.0)
        self.paramC02.setMaximum(1.0)
        self.paramC02.setDecimals(2)
        self.paramC02.setValue(0.0)

        OptionN2 = QPushButton("%N2")
        OptionN2.pressed.connect(partial(self.setpar,3))
        self.paramN2 = QDoubleSpinBox()
        self.paramN2.setMinimum(0.0)
        self.paramN2.setMaximum(1.0)
        self.paramN2.setDecimals(2)
        self.paramN2.setValue(0.79)

        OptionTexhale = QPushButton("T Exhale (K)")
        OptionTexhale.pressed.connect(partial(self.setpar,4))
        self.paramTexhale = QDoubleSpinBox()
        self.paramTexhale.setMinimum(0.0)
        self.paramTexhale.setMaximum(500.0)
        self.paramTexhale.setDecimals(1)
        self.paramTexhale.setValue(293)

        OptionH20 = QPushButton("H20")
        OptionH20.pressed.connect(partial(self.setpar,5))
        self.paramH20 = QDoubleSpinBox()
        self.paramH20.setMinimum(0.0)
        self.paramH20.setMaximum(1.0)
        self.paramH20.setDecimals(2)
        self.paramH20.setValue(0.0)
        
        self.AlarmBox = QGroupBox("Alarm Values")
        
        OptionLow = QPushButton("Low Flow")
        OptionLow.pressed.connect(partial(self.setpar,6))
        self.paramLow = QDoubleSpinBox()
        self.paramLow.setMaximum(1000)
        self.paramLow.setDecimals(1)
        self.paramLow.setValue(300)

        OptionHigh = QPushButton("High Flow")
        OptionHigh.pressed.connect(partial(self.setpar,7))
        self.paramHigh = QDoubleSpinBox()
        self.paramHigh.setMaximum(1000)
        self.paramHigh.setDecimals(1)
        self.paramHigh.setValue(700)

        OptionDiff = QPushButton("Percent Difference")
        OptionDiff.pressed.connect(partial(self.setpar,8))
        self.paramDiff = QDoubleSpinBox()
        self.paramDiff.setValue(0.5)
        self.paramH20.setMinimum(0.0)
        self.paramH20.setMaximum(1)
        self.paramDiff.setDecimals(2)

        #self.ScaleBox = QGroupBox("Auto-Scaling ON/OFF")

        #OptionALL = QPushButton("Auto-Scale")
        #OptionALL.pressed.connect(partial(self.setpar,0))

        layout1 = QGridLayout()

        layout1.addWidget(Option02,0,2)
        layout1.addWidget(self.param02,0,0,1,2)

        layout1.addWidget(OptionC02,1,2)
        layout1.addWidget(self.paramC02,1,0,1,2)

        layout1.addWidget(OptionN2,2,2)
        layout1.addWidget(self.paramN2,2,0,1,2)

        layout1.addWidget(OptionTexhale,3,2)
        layout1.addWidget(self.paramTexhale,3,0,1,2)

        layout1.addWidget(OptionH20,4,2)
        layout1.addWidget(self.paramH20,4,0,1,2)
        

        layout2 = QGridLayout()

        layout2.addWidget(OptionLow,5,2)
        layout2.addWidget(self.paramLow,5,0,1,2)

        layout2.addWidget(OptionHigh,6,2)
        layout2.addWidget(self.paramHigh,6,0,1,2)

        layout2.addWidget(OptionDiff,7,2)
        layout2.addWidget(self.paramDiff,7,0,1,2)

        #layout3 = QGridLayout()

        #layout3.addWidget(OptionALL,8,0,1,3)

        #Intialize box

        self.initBox = QGroupBox("Initialize Parameters")

        layout4 = QGridLayout()

        OptionInit = QPushButton("Initialize")
        OptionInit.pressed.connect(self.init)

        layout4.addWidget(OptionInit, 9, 0, 1, 3)

        self.ParamBox.setLayout(layout1)
        self.AlarmBox.setLayout(layout2)
        #self.ScaleBox.setLayout(layout3)
        self.initBox.setLayout(layout4)

    #PLOT CODE FROM: https://www.learnpyqt.com/courses/graphics-plotting/plotting-matplotlib/
    def makePlotBox(self):
        self.PlotBox = QGroupBox("Updates")

        self.canvas1 = MplCanvas(self, width=7, height=5, dpi=100)
        toolbar = NavigationToolbar(self.canvas1, self)
        #data to plot
        n_data = 100 #num points on x axis
        self.xdata = numpy.arange(-10, 0, 0.1).tolist()
        self.rawdata = [i for i in self.xdata]
        self._plot_ref = None
        self.update_plots()

        layout = QVBoxLayout()
        layout.addWidget(toolbar)
        layout.addWidget(self.canvas1)
        self.PlotBox.setLayout(layout)

        # Setup a timer to trigger the redraw by calling update_plot.
        # For this applicaton - might be better to have a thread that updates whenever it receives new information --- That would be Qthreadpool I think

        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start()

    def update_plots(self):
        #read from serial port
        string = ser.readline()
        msg = string.decode('utf-8')
        #msg = "flow rate, Inhale, Exhale"
        dataList = msg.split(",")
        #dataList = [flow rate , Inhale, Exhale] as a list
        try:
            number = float(dataList[0])*60 #slps -> SLPM
            self.rawdata = self.rawdata[1:] + [number]

            #update flowrate textbox
            self.volumeInBox.setText("Tidal Volume Inhale (ml): " + dataList[1])
            self.volumeOutBox.setText("Tidal Volume Exhale (ml): " + dataList[2])

            # Drop off the first y element, append a new one.
            self.ydata = self.rawdata

            #check if plot has been started
            if self._plot_ref is None:
                #initalize the plot refrence
                #.plot returns a list of line refrences but we only have 1 line so use first ref
                plot_refs = self.canvas1.axes.plot(self.xdata, self.ydata, 'r')
                self._plot_ref = plot_refs[0]
            else:
                #if refrence intialized append new data
                self._plot_ref.set_ydata(self.ydata)

            #auto-scaling ###### Currenly not working
            #if scale:
                #self.canvas1.axes.set_autoscale_on(True)
            #else:
                #self.canvas1.axes.set_ybound(lower=-30, upper=30)

            #replacment for auto-scaling
            self.canvas1.axes.set_ybound(lower=-60, upper=60)
            
            #Figure 1 labels
            self.canvas1.axes.set_xlabel("Time (Sec)")
            self.canvas1.axes.set_ylabel("Minute Volume (SLPM)")

            # Trigger the canvas to update and redraw.
            self.canvas1.draw()
            #write data to CSV file
            with open(os.path.join(path, filename), 'a', newline='') as file:
                writer = csv.writer(file)
                now = datetime.now()
                writer.writerow([number, float(dataList[1]), float(dataList[2]), now.strftime("%H:%M:%S")])
        except:
            ser.flushInput()

    #I happen when you click a button
    def setpar(self, par2set):
        sendOut = "update parameter " + str(par2set)
        switch = { 0: 42,
                    1: str(self.param02.value()) + ",",
                    2: str(self.paramC02.value()) + ",",
                    3: str(self.paramN2.value()) + ",",
                    4: str(self.paramTexhale.value()) + ",",
                    5: str(self.paramH20.value()) + ",",
                    6: str(self.paramLow.value()) + ",",
                    7: str(self.paramHigh.value()) + ",",
                    8: str(self.paramDiff.value()) + "$",
                   }
        self.sendOut = switch.get(par2set)
        if self.sendOut == 42:
            #print("Auto Scaling")
            global scale
            scale = not scale
        else:
            self.sendData(self.sendOut)

    #send data to serial port
    def sendData(self, msg):
        #print("Sending", msg)
        ser.write(msg.encode())

    def init(self):
        #####make this a button that we can press like initialize used to be####
        self.sendData(str(self.param02.value()) + ",") # O2
        self.sendData(str(self.paramC02.value()) + ",") # CO2
        self.sendData(str(self.paramN2.value()) + ",") #N2
        self.sendData(str(self.paramTexhale.value()) + ",") #T Air Exhale
        self.sendData(str(self.paramH20.value()) + ",") #H20
        self.sendData(str(self.paramLow.value()) + ",") # Low Flow error bound
        self.sendData(str(self.paramHigh.value()) + ",") # High Flow error bound
        self.sendData(str(self.paramDiff.value()) + "$") # % difference error bound

        
        
if __name__ == '__main__':

    app = QApplication([])

    wind = RespiratorDisplay()
    wind.show()
    sys.exit(app.exec_())
