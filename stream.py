import serial
import time
import numpy as np


class grbl():
    def __init__(self,com_port):
        self.s = serial.Serial(com_port,115200)
        self.s.write("\r\n\r\n".encode()) # Wake up grbl
        time.sleep(2)   # Wait for grbl to initialize 
        self.s.flushInput()  # Flush startup text in serial input
        self.cond = True
        self.first_x = 0
        self.first_y = 0

    def scale(self, xpos, ypos, scale):
        if self.first_x == 0:
            self.first_x = float(xpos)
            self.first_y = float(ypos)

        self.x_new = (float(xpos)-self.first_x)*scale
        self.y_new = (float(ypos)-self.first_y)*scale

        return str(self.x_new), str(self.y_new)

    def grbl_stream(self,xpos,ypos):
        xpos, ypos = self.scale(xpos, ypos, -30)

        if self.cond is True:
            line = '$X\nG92 X0 Y0 Z0\nG17 G21 G90 G94 G54\nG01 X0 Y0 Z0 F6000\n'
            self.cond = False
        else:
            line = 'X{} Y{}'.format(xpos,ypos) #input x,y position from arduino
        self.s.flushInput()  # Flush startup text in serial input
        li = line.strip() # Strip all EOL characters for consistency
        self.s.write(li.encode()+ '\n'.encode()) # Send g-code block to grbl
        grbl_out = self.s.readline() # Wait for grbl response with carriage return
        print(line)
        print( ' : ', grbl_out.strip())


class hapkit():
    def __init__(self, com_port):
        self.s2 = serial.Serial(com_port,9600)
        self.xpos = 0
        self.ypos = 0

    def hapkit_stream(self):
        xypos = self.s2.readline().decode().strip()
        self.xpos = xypos.split('a')[1]
        self.ypos = xypos.split('a')[0]




#for linux '/dev/ttyACM0', for windows 'COM1'
grbl = grbl('COM10')#'/dev/ttyACM1')
hapkit = hapkit('COM8')#'/dev/ttyACM0')

while True:
    grbl.grbl_stream(hapkit.xpos,hapkit.ypos)
    hapkit.hapkit_stream()

    
    
    