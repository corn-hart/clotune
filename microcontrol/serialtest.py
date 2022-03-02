# Importing Libraries
import serial
import time
arduino = serial.Serial(port='COM5', baudrate=9800, timeout=.1)
def write_read():
    time.sleep(0.05)
    data = arduino.readline()
    data2 = data.decode()
    return data2
while True:
    value = write_read()
    print(value) # printing the value