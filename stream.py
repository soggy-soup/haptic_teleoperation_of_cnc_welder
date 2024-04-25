import serial
import time 
import keyboard #pip install keyboard


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
        xpos, ypos = self.scale(xpos, ypos, -25.4)

        if self.cond ==0 :
            line = '$X\nG92 X0 Y0 Z0\nG17 G21 G90 G94 G54\nG01 X0 Y0 Z0 F6000\n'
            self.cond = 1
        else:
            line = 'X{} Y{}'.format(ypos,xpos) #input x,y position from arduino

        li = line.strip() # Strip all EOL characters for consistency
        self.s.write(li.encode()+ '\n'.encode()) # Send g-code block to grbl
        self.s.flushInput()  # Flush startup text in serial input
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




grbl1 = grbl('COM10')#'/dev/ttyACM1')
hapkit1 = hapkit('COM8')#'/dev/ttyACM0')


while True:

    while keyboard.is_pressed('c') == True:
        print('CLUTCHING BABY')
        hapkit1.hapkit_stream()
        grbl1.first_x = float(hapkit1.xpos)
        grbl1.first_y = float(hapkit1.ypos)
        grbl1.cond = 0

    grbl1.grbl_stream(hapkit1.xpos,hapkit1.ypos)
    hapkit1.hapkit_stream()


