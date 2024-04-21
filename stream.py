import serial
import time

xpos = 0
ypos = 0

class grbl_write():
    def __init__(self,com_port):
            
        self.s = serial.Serial(com_port,115200)
        self.s.write("\r\n\r\n".encode()) # Wake up grbl
        time.sleep(2)   # Wait for grbl to initialize 
        self.s.flushInput()  # Flush startup text in serial input
        self.cond = True
    
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
            line = 'X{} Y{}'.format(xpos,ypos) #input x,y position from arduino
        
        li = line.strip() # Strip all EOL characters for consistency
        print('Sending: ', li)
        self.s.write(li.encode()+ '\n'.encode()) # Send g-code block to grbl
        grbl_out = self.s.readline() # Wait for grbl response with carriage return
        print(' : ', grbl_out.strip())  
    
    
class hapkit_read():
    def __init__(self, com_port):
        self.s2 = serial.Serial(com_port,115200)
    
    def hapkit_stream(self):
        xypos = self.s2.readline().decode().strip()
        xpos = xypos.split('a')[1]
        ypos = xypos.split('a')[0]
        return xpos, ypos

      

grbl = grbl_write('COM10')    
hapkit = hapkit_read('COM8')

while True:
    grbl.grbl_stream(xpos,ypos)
    xpos,ypos = hapkit.hapkit_stream()
    print(xpos,ypos)
    
    
    
    