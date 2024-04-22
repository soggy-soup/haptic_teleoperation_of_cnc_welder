import serial
import time
import numpy as np
xpos = 0
ypos = 0

class grbl():
    def __init__(self,com_port):
            
        self.s = serial.Serial(com_port,115200)
        self.s.write("\r\n\r\n".encode()) # Wake up grbl
        time.sleep(2)   # Wait for grbl to initialize 
        self.s.flushInput()  # Flush startup text in serial input
        self.cond = True
        self.machine_x = 0
        self.machine_y = 0
    
    def scale(self, xpos, ypos, scale):
        self.x_new = (float(xpos)-3.27)*scale
        self.y_new = (float(ypos)-1.36)*scale

        return str(self.x_new), str(self.y_new)

    def grbl_stream(self,xpos,ypos):
        xpos, ypos = self.scale(xpos, ypos, -100)

        if self.cond is True:
            line = '$X\nG92 X0 Y0 Z0\nG17 G21 G90 G94 G54\nG01 X0 Y0 Z0 F6000\n'
            self.cond = False
        else:
            line = 'X{} Y{}?'.format(xpos,ypos) #input x,y position from arduino
        
        li = line.strip() # Strip all EOL characters for consistency
        self.s.write(li.encode()+ '\n'.encode()) # Send g-code block to grbl
        
        #Position Feedback from GRBL
        grbl_out = self.s.readline().decode().strip() #raw output from GRBL
        grbl_pos = grbl_out.split('|') #first split
        if len(grbl_pos) > 1: #ensure that position data is being sent
            grbl_pos = grbl_pos[1].split(':')[1].split(',')
            self.machine_x = grbl_pos[0]
            self.machine_y = grbl_pos[1]

    
            
        
        
         
    
    
class hapkit():
    def __init__(self, com_port):
        self.s2 = serial.Serial(com_port,115200)
        self.xpos = 0
        self.ypos = 0
    
    def hapkit_stream(self,machine_x,machine_y):
        xypos = self.s2.readline().decode().strip()
        self.xpos = xypos.split('a')[0]
        self.ypos = xypos.split('a')[1]
        hapkitmachinepos = xypos.split('a')[2]
        print("hapkitmachineposition: ",hapkitmachinepos)
        machine_xypos = '{}a{}'.format(machine_x,machine_y)
        self.s2.write(machine_xypos.encode()+ '\n'.encode())
        

      
#for linux '/dev/ttyACM0', for windows 'COM1'
grbl = grbl('/dev/ttyACM0')  
hapkit = hapkit('/dev/ttyACM1')

while True:
    grbl.grbl_stream(hapkit.xpos,hapkit.ypos)
    hapkit.hapkit_stream(grbl.machine_x,grbl.machine_y)
    
    #print(hapkit.xpos,hapkit.ypos,"a",grbl.machine_x,grbl.machine_y)
    
    
    