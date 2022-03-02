from pyo import *
import serial
import time
arduino = serial.Serial(port='COM5', baudrate=9800, timeout=.1)


   
    
def write_read():
    time.sleep(0.1)
    data = arduino.readline()
    data2 = data.decode()
    return data2


    
s = Server().boot()
s.start()
beat = Beat(time=.125, taps=16, w1=[90,80], w2=50, w3=35, poly=1).play()
beat.out()
sound = Sine()


sound.out()



while True:
    
    value = write_read()
    if value:
        value2 = int(value)
        sound.setFreq(value2)
        

s.stop()