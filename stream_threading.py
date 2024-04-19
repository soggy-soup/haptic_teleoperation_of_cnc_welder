import serial
import time
import threading


c = threading.Condition()
xypos = '0a0'
last_xypos = xypos
flag = 0


class to_grbl_thread(threading.Thread):
    def __init__(self,com_port):
        threading.Thread.__init__(self)
        self.com_port = com_port
        
        self.s = serial.Serial(com_port,115200)
        self.s.write("\r\n\r\n".encode()) # Wake up grbl
        time.sleep(2)   # Wait for grbl to initialize 
        self.s.flushInput()  # Flush startup text in serial input
        self.firstline = '$X\nG92 X0 Y0 Z0\nG17 G21 G90 G94 G54\nG01 X0 Y0 Z0 F5000\n' 
    
    def run(self):
        global xypos
        
        while True:
            c.acquire()
            xpos = 0
            ypos = 0 #parse 0a0 data later cuz im lazy
            line = 'X{} Y{}'.format(xpos,ypos) #input x,y position from arduino
            li = line.strip() # Strip all EOL characters for consistency
            print('Sending: ', li)
            self.s.write(li.encode()+ '\n'.encode()) # Send g-code block to grbl
            grbl_out = self.s.readline() # Wait for grbl response with carriage return
            print(' : ', grbl_out.strip())       
            c.notify_all
            c.release()
 


class from_hapkit_thread(threading.Thread):
    def __init__(self,com_port):
        threading.Thread.__init__(self)
        self.com_port = com_port
        self.s = serial.Serial(com_port,115200)
    
    
    def run(self):
        global xypos
        
        while True:
            c.acquire()
            xypos = self.s.readline().decode().strip()          
                 
            c.notify_all
            c.release()
 


grbl = to_grbl_thread("GRBL STREAM THREAD")
hapkit = from_hapkit_thread("HAPKIT READ POSITION THREAD")

grbl.start()
hapkit.start()

grbl.join()
hapkit.join()