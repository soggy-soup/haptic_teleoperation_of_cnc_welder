import tkinter as tk
from tkinter import *
import serial
import time 
import keyboard #pip install keyboard


class window(tk.Tk):
    def __init__(self):
        super().__init__()
        #initialize window properties
        self.bind('<Escape>', lambda e: self.quit())
        self.title("CNC Welder")
        self.resizable(width=True, height=True)
        width = 1000
        height = 1000
        win = tk.Frame(self,width=width,height=height)
        win.pack()
        self.C = tk.Canvas(win, bg="black", height = height, width = width)
        self.C.pack()
        self.line = self.C.create_line(60,60,900,800,fill='blue')
        
       #for linux '/dev/ttyACM0', for windows 'COM1'
        '''
        grbl = grbl('COM10')#'/dev/ttyACM1')
        hapkit = hapkit('COM8')#'/dev/ttyACM0')

        while keyboard.is_pressed('c') == True:
            print('CLUTCHING BABY')
            grbl.first_x = 0
            grbl.cond = 2
        
        grbl.grbl_stream(hapkit.xpos,hapkit.ypos)
        hapkit.hapkit_stream()
        '''
        #self.cursor_draw(hapkit.xpos,hapkit.ypos)
        self.cursor_draw(250,250)

    def cursor_draw(self,x_cur,y_cur):
        cursor = self.C.create_oval(x_cur+5,y_cur+5,x_cur-5,y_cur-5, fill = 'red')

    #Class for hapkit
    #class for GRBL
    class grbl():
        def __init__(self,com_port):
            self.s = serial.Serial(com_port,115200)
            self.s.write("\r\n\r\n".encode()) # Wake up grbl
            time.sleep(2)   # Wait for grbl to initialize 
            self.s.flushInput()  # Flush startup text in serial input
            self.cond = 0
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
            xpos, ypos = self.scale(xpos, ypos, -10)

            if self.cond ==0 :
                line = '$X\nG92 X0 Y0 Z0\nG17 G21 G90 G94 G54\nG01 X0 Y0 Z0 F6000\n'
                self.cond = 1
            elif self.cond==2:
                line = 'G92 X0 Y0 Z0\nG17 G21 G90 G94 G54\nG01 X0 Y0 Z0 F6000\n'
                self.cond = 1
            else:
                line = 'X{} Y{}?'.format(xpos,ypos) #input x,y position from arduino

            li = line.strip() # Strip all EOL characters for consistency
            self.s.write(li.encode()+ '\n'.encode()) # Send g-code block to grbl
        #    grbl_out = self.s.readline() # Wait for grbl response with carriage return
        #    print( ' : ', grbl_out.strip())
    
    class hapkit():
        def __init__(self, com_port):
            self.s2 = serial.Serial(com_port,115200)
            self.xpos = 0
            self.ypos = 0

        def hapkit_stream(self):
            xypos = self.s2.readline().decode().strip()
            self.xpos = xypos.split('a')[0]
            self.ypos = xypos.split('a')[1]
            print(xypos)



gui = window()
gui.mainloop()

