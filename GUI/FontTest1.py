import matplotlib
matplotlib.use('Qt5Agg')


from PySide2.QtCore import QDateTime, Qt, QTimer
from PySide2.QtWidgets import (QMainWindow, QApplication, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QDoubleSpinBox, QVBoxLayout, QWidget, QAction)

import numpy
import sys
import os
import random
from functools import partial

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

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
filename = "DATA_" + date_time + ").csv"

#universal vraibles
scale = 60
alarm = False
init = False

#create file and set headings
with open(os.path.join(path, filename), 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Volume Flow Rate","Filtered Pressure Inhale","Filtered Pressure Exhale","Static Pressure","Volume Inhale","Volume Exhale","Alarm Code", "Timestamp"])

#print available com ports
comPorts = list(serial.tools.list_ports.comports())
for device in comPorts:
    print(device)


#Ask for user input, loop until valid input is found
valid_input = False
port = input("Serial port: ")
deviceName = ""

while not valid_input:
    try:
        ser = serial.Serial(port, timeout = 3, baudrate = 115200)
        valid_input = True
        deviceName = port
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

class Window(QMainWindow):
    def __init__(self):
        super(Window, self).__init__()

        mainWidget = RespiratorDisplay()

        # Retrieve the MenuBar widget from the parent QDialog class.
        mainMenu = self.menuBar()
        # Add a new menu item to it: the View menu.
        viewMenu = mainMenu.addMenu('&View')

        fontSizeMenu = viewMenu.addMenu('&Font Size')

        fontSizeOptions = [12, 14, 16, 18, 20, 22]
        for index, fontSize in enumerate(fontSizeOptions):
            actionName = "%spx" % (fontSize)
            action = fontSizeMenu.addAction(actionName)
            action.setData(fontSize)
            action.triggered.connect(partial(self.setFontSize, action))

        self.setCentralWidget(mainWidget)
        self.setWindowTitle("Board Info")

    def setFontSize(self, actionItem):
        data = actionItem.data()
        setFontSize(data)



#ACTUAL GUI CLASS
class RespiratorDisplay(QWidget):
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
        reset = QPushButton("Reset Alarm")
        reset.pressed.connect(partial(self.sendData,"R"))
        stop = QPushButton("Stop")
        stop.pressed.connect(partial(self.sendData,"X"))
        start = QPushButton("Start")
        start.pressed.connect(partial(self.sendData,"S"))
        bottomLayout.addWidget(reset, 1, 1)
        bottomLayout.addWidget(start, 1, 2)
        bottomLayout.addWidget(stop, 1, 3)
        bottomGroup.setLayout(bottomLayout)

        #show flowrate values
        self.makeUpdateBox()

        mainLayout = QGridLayout()
        mainLayout.addLayout(topLayout, 0, 0, 1, 2)
        mainLayout.addWidget(self.PlotBox, 1, 0, 2, 1)
        mainLayout.addWidget(self.ParamBox, 1, 1, 1, 1)
        mainLayout.addWidget(self.AlarmBox, 2, 1, 1, 1)
        mainLayout.addWidget(self.ScaleBox, 3, 1, 1, 1)
        mainLayout.addWidget(self.initBox, 4, 1, 1, 1)
        mainLayout.addWidget(bottomGroup, 4, 0, 1, 1)
        mainLayout.addWidget(self.UpdateBox, 3, 0, 1, 1)

        self.setLayout(mainLayout)

    def makeUpdateBox(self):
        self.UpdateBox = QGroupBox("Updates")

        self.volumeInBox = QLabel("Tidal Inhale Volume (ml): ")
        self.volumeOutBox = QLabel("Tidal Exhale Volume (ml): ")
        self.staticPressBox = QLabel("Static Pressure (psi): ")
        self.AlarmCodeBox = QLabel("Alarm Cause: ")

        updateLayout = QGridLayout()

        updateLayout.addWidget(self.volumeInBox)
        updateLayout.addWidget(self.volumeOutBox)
        updateLayout.addWidget(self.staticPressBox)
        updateLayout.addWidget(self.AlarmCodeBox)

        self.UpdateBox.setLayout(updateLayout)        

    def makeParamBox(self):

        self.ParamBox = QGroupBox("Inputs")

        Option02 = QPushButton("Inhale %O2")
        Option02.pressed.connect(partial(self.setpar,1))
        self.param02 = QDoubleSpinBox()
        #set range with this method
        self.param02.setMinimum(0.0)
        self.param02.setMaximum(1.0)
        self.param02.setDecimals(2)
        self.param02.setValue(0.21)
        Option02.setFont("arial")
        

        OptionO2Exhale = QPushButton("Exhale %O2")
        OptionO2Exhale.pressed.connect(partial(self.setpar,2))
        self.paramO2Exhale = QDoubleSpinBox()
        self.paramO2Exhale.setMinimum(0.0)
        self.paramO2Exhale.setMaximum(1.0)
        self.paramO2Exhale.setDecimals(2)
        self.paramO2Exhale.setValue(0.0)

        OptionCO2Exhale = QPushButton("Exhale %CO2")
        OptionCO2Exhale.pressed.connect(partial(self.setpar,3))
        self.paramCO2Exhale = QDoubleSpinBox()
        self.paramCO2Exhale.setMinimum(0.0)
        self.paramCO2Exhale.setMaximum(1.0)
        self.paramCO2Exhale.setDecimals(2)
        self.paramCO2Exhale.setValue(0.79)

        OptionN2Exhale = QPushButton("Exhale %N2")
        OptionN2Exhale.pressed.connect(partial(self.setpar,4))
        self.paramN2Exhale = QDoubleSpinBox()
        self.paramN2Exhale.setMinimum(0.0)
        self.paramN2Exhale.setMaximum(500.0)
        self.paramN2Exhale.setDecimals(1)
        self.paramN2Exhale.setValue(0.0)

        OptionH20 = QPushButton("Exhale %H20")
        OptionH20.pressed.connect(partial(self.setpar,5))
        self.paramH20 = QDoubleSpinBox()
        self.paramH20.setMinimum(0.0)
        self.paramH20.setMaximum(1.0)
        self.paramH20.setDecimals(2)
        self.paramH20.setValue(0.0)
        
        self.AlarmBox = QGroupBox("Alarm Values")
        
        OptionLow = QPushButton("Low Volume (ml)")
        OptionLow.pressed.connect(partial(self.setpar,6))
        self.paramLow = QDoubleSpinBox()
        self.paramLow.setMaximum(1000)
        self.paramLow.setDecimals(1)
        self.paramLow.setValue(300)

        OptionHigh = QPushButton("High Volume (ml)")
        OptionHigh.pressed.connect(partial(self.setpar,7))
        self.paramHigh = QDoubleSpinBox()
        self.paramHigh.setMaximum(4000)
        self.paramHigh.setDecimals(1)
        self.paramHigh.setValue(700)

        OptionDiff = QPushButton("Percent Difference (0-1)")
        OptionDiff.pressed.connect(partial(self.setpar,8))
        self.paramDiff = QDoubleSpinBox()
        self.paramDiff.setValue(0.5)
        self.paramH20.setMinimum(0.0)
        self.paramH20.setMaximum(1)
        self.paramDiff.setDecimals(2)

        self.ScaleBox = QGroupBox("Scale")
        
        OptionALL = QPushButton("Change Scale")
        OptionALL.pressed.connect(partial(self.setpar,0))
        self.paramScale = QDoubleSpinBox()
        self.paramScale.setMaximum(1000)
        self.paramScale.setDecimals(1)
        self.paramScale.setValue(scale)

        layout1 = QGridLayout()

        layout1.addWidget(Option02,0,2)
        layout1.addWidget(self.param02,0,0,1,2)

        layout1.addWidget(OptionO2Exhale,1,2)
        layout1.addWidget(self.paramO2Exhale,1,0,1,2)

        layout1.addWidget(OptionCO2Exhale,2,2)
        layout1.addWidget(self.paramCO2Exhale,2,0,1,2)

        layout1.addWidget(OptionN2Exhale,3,2)
        layout1.addWidget(self.paramN2Exhale,3,0,1,2)

        layout1.addWidget(OptionH20,4,2)
        layout1.addWidget(self.paramH20,4,0,1,2)
        

        layout2 = QGridLayout()

        layout2.addWidget(OptionLow,5,2)
        layout2.addWidget(self.paramLow,5,0,1,2)

        layout2.addWidget(OptionHigh,6,2)
        layout2.addWidget(self.paramHigh,6,0,1,2)

        layout2.addWidget(OptionDiff,7,2)
        layout2.addWidget(self.paramDiff,7,0,1,2)

        layout3 = QGridLayout()
        layout3.addWidget(OptionALL,8,2)
        layout3.addWidget(self.paramScale,8,0,1,2)

        #Intialize box

        self.initBox = QGroupBox("Initialize Parameters")

        layout4 = QGridLayout()

        OptionInit = QPushButton("Calibrate")
        OptionInit.pressed.connect(self.init)

        layout4.addWidget(OptionInit, 9, 0, 1, 3)

        self.ParamBox.setLayout(layout1)
        self.AlarmBox.setLayout(layout2)
        self.ScaleBox.setLayout(layout3)
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
        global alarm
        global scale
        global init
        
        if init:
            #read from serial port
            string = ser.readline()
            print(string)
            msg = string.decode('utf-8')
            dataList = msg.split(",")

        else:
            dataList = [0, 0, 0]

            #dataList = [flowrate in, pressure in, pressure ex, alarm cause, flowrate in, flowrate ex, vol in, vol out]
             
        try:
            number = float(dataList[1])*60
            self.rawdata = self.rawdata[1:] + [number]

            #update flowrate textbox
            self.volumeInBox.setText("Tidal Volume Inhale (ml): " + dataList[6])
            self.volumeOutBox.setText("Tidal Volume Exhale (ml): " + dataList[7])
            self.staticPressBox.setText("Static Pressure (psi): " + dataList[5])

            if (float(dataList[0])==1):
                self.AlarmCodeBox.setText("Alarm Cause: Percent Difference")
            if (float(dataList[0])==2):
                self.AlarmCodeBox.setText("Alarm Cause: Low Flow")
            if (float(dataList[0])==3):
                self.AlarmCodeBox.setText("Alarm Cause: High Flow")
            if (float(dataList[0])==100):
                self.AlarmCodeBox.setText("Callibrating System: Make sure no Flow")
            if (float(dataList[0])==101):
                self.AlarmCodeBox.setText("Finished Calibration: Press 'Start' when ready")
            if (float(dataList[0])==999):
                self.AlarmCodeBox.setText("Alarm Cause: No Flow")
            if (float(dataList[0])==-1):
                self.AlarmCodeBox.setText("Alarm Cause: ---")

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

            #replacment for auto-scaling
            self.canvas1.axes.set_ybound(lower=-1*scale, upper=scale)
            
            #Figure 1 labels
            self.canvas1.axes.set_xlabel("Time (Sec)")
            self.canvas1.axes.set_ylabel("Minute Volume (SLPM)")

            # Trigger the canvas to update and redraw.
            self.canvas1.draw()
            #write data to CSV file
            with open(os.path.join(path, filename), 'a', newline='') as file:
                writer = csv.writer(file)
                now = datetime.now()
                writer.writerow([number, float(dataList[2]), float(dataList[3]), float(dataList[5]), float(dataList[6]), float(dataList[7]), float(dataList[0]), now.strftime("%H:%M:%S")])
                
        except:
            ser.flushInput()

    #I happen when you click a button
    def setpar(self, par2set):
        global scale
        sendOut = "update parameter " + str(par2set)
        switch = { 0: 42,
                    1: str(self.param02.value()) + ",",
                    2: str(self.paramO2Exhale.value()) + ",",
                    3: str(self.paramCO2Exhale.value()) + ",",
                    4: str(self.paramN2Exhale.value()) + ",",
                    5: str(self.paramH20.value()) + ",",
                    6: str(self.paramLow.value()) + ",",
                    7: str(self.paramHigh.value()) + ",",
                    8: str(self.paramDiff.value()) + "$",
                   }
        self.sendOut = switch.get(par2set)
        if par2set == 0:
            scale = int(self.paramScale.value())


    #send data to serial port
    def sendData(self, msg):
        print("Sending", msg)
        ser.write(msg.encode())

    def init(self):
        global init
        init = True 
        self.sendData(str(self.param02.value()) + ",") # O2 Inhale
        self.sendData(str(self.paramO2Exhale.value()) + ",") # O2 Exhale
        self.sendData(str(self.paramCO2Exhale.value()) + ",") #CO2 exhale
        self.sendData(str(self.paramN2Exhale.value()) + ",") #N2 Air Exhale
        self.sendData(str(self.paramH20.value()) + ",") #H20 exhale
        self.sendData(str(self.paramLow.value()) + ",") # Low Flow error bound
        self.sendData(str(self.paramHigh.value()) + ",") # High Flow error bound
        self.sendData(str(self.paramDiff.value()) + "$") # % difference error bound

def setFontSize(newFontSize):
    curFont = app.font()
    curFont.setPointSize(newFontSize)
    app.setFont(curFont)

if __name__ == '__main__':

    app = QApplication([])

    wind = Window()
    wind.show()

    sys.exit(app.exec_())