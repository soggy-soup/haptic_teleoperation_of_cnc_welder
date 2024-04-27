import serial
import time 
import keyboard #pip install keyboard
import csv

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
        xpos, ypos = self.scale(xpos, ypos, 20.0)

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
        self.recording = False  # Flag to control recording state

    def hapkit_stream(self):
        xypos = self.s2.readline().decode().strip()
        self.xpos = xypos.split('a')[0]
        self.ypos = xypos.split('a')[1]
        if self.recoridng:
            print(f"xpos: {self.xpos}, ypos: {self.ypos}")
        

    def toggle_recording(self):
        self.recording = not self.recording
        return self.recording

    def write_to_csv(self, filename):
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['xpos', 'ypos'])  # Write header
            while self.recording:
                self.hapkit_stream()
                writer.writerow([self.xpos, self.ypos])




##grbl1 = grbl('COM10')#'/dev/ttyACM1')
hapkit1 = hapkit('COM8')#'/dev/ttyACM0')



while True:
    if keyboard.is_pressed('space'):
        recording_state = hapkit1.toggle_recording()
        if recording_state:
            print("Recording started...")
            hapkit1.write_to_csv('hapkit_data.csv')
            time.sleep(10)
            print("Recording stopped.")
            # Reset recording flag and wait for spacebar release

while True:
    
    ##while keyboard.is_pressed('c') == True:
    #    print('CLUTCHING BABY')
    #    hapkit1.hapkit_stream()
    #    grbl1.first_x = float(hapkit1.xpos)
    #    grbl1.first_y = float(hapkit1.ypos)
    #    grbl1.cond = 0

    #grbl1.grbl_stream(hapkit1.xpos,hapkit1.ypos)
    hapkit1.hapkit_stream()


