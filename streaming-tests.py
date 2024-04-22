import serial 
import time 
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1) 
def write_read(x): 
	arduino.write(bytes(x, 'utf-8')) 
	time.sleep(0.05) 
	data = arduino.readline() 
	return data 


while True: 
    print(arduino.readline().decode().strip().split("a"))