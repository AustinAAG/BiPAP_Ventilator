import serial
import serial.tools.list_ports

import csv
import time
from datetime import datetime

#generate file name with current date and time
now = datetime.now()
date_time = now.strftime("%m")+"."+now.strftime("%d")+"."+now.strftime("%y")+"_("+now.strftime("%H_%M_%S")

filename = "BiPAP_" + date_time + ").csv"

with open(filename, 'a+', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["pressureIn" ,"pressureEx" , "pressureIn_f" ,"pressureEx_f" , "FlowrateIn" ,"FlowrateEx" ,"Inhale volume" ,"Exhale Volume", "Timestamp"])
    

#print available device names
comPorts = list(serial.tools.list_ports.comports())
for device in comPorts:
    print(device)

#Ask for user input, loop until valid input is found
valid_input = False
port = input("Serial port: ")

while not valid_input:
    try:
        ser = serial.Serial(port, timeout = 3, baudrate = 115200)
        valid_input = True
        print("Device found!")
        print(ser)
    except:
        print("No device found!")
        valid_input = False
        port = input("Reenter serial port: ")        




##LEAVE THIS ON FOR AUSTIN




#collect and print data
rawdata = []
list = [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0]
while True:
    string = ser.readline()
    #print(string)
    msg = string.decode('utf-8')
    list = msg.split(",")
    #rawdata.append(str(ser.readline()))
    #print(msg.decode(ser.readline()))
    now = datetime.now()
    #print(msg)
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        now = datetime.now()
        writer.writerow([float(list[0]), float(list[1]), float(list[2]), float(list[3]), float(list[4]), float(list[5]), float(list[6]), float(list[7]), now.strftime("%H:%M:%S")])
    try:        
        number = msg
        print(list)
        #print(now.strftime("%H:%M:%S.%f")[:-3])
        #ser.flushInput()
    except:
        print('missed')
        #ser.flushInput()
    
    

        
