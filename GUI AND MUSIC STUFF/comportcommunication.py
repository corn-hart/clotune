#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 15 15:25:42 2022

@author: Radu
"""

# Importing Libraries
import serial
import time
import copy
import numpy
serial_buffer = []

clo_board = serial.Serial(port='/dev/tty.usbmodem14203', baudrate=115200)


# Store calibration values for all sensors
no_sensors = 4
# Dictates the number of readings needed for each sensor to complete its calibration
no_passes = 4
sens_calib = numpy.zeros(no_sensors)
calib_bulb = numpy.zeros(no_sensors)
# Structure of calibration vector [flex1, flex2, accel1_x, accel1_y, accel1_z, accel2_x, accel2_y, accel2_z, ultras1]
sens_labels = ['f1', 'f2', 'Ax', 'Ay']

# SENSOR CALIBRATION
while (int(numpy.prod(calib_bulb)) != no_passes ** no_sensors):
    # Read, decode, strip serial message
    message = copy.deepcopy(clo_board.read_until())
    decoded_message = message.decode().strip()
    print("MSG: ",decoded_message)

    # If decoded message is not empty
    if decoded_message:
        
        # If a sensor is triggered, calibrate that sensor
        if (decoded_message[0] != 'b'):
            
            # Save the reference value for each sensor
            for jj in range (no_sensors):
                # If a certain sensor sends data, record its value
                if (decoded_message[0:2] == sens_labels[jj]) and (calib_bulb[jj] < no_passes):
                    # Add the read value to the averaging function
                    sens_calib[jj] += int(decoded_message[3:])
                    # Calibrate each sensor "no_passes" times
                    calib_bulb[jj] += 1
                    break
                
        print('VEC: ',calib_bulb)
        print('PRODUCT: ',int(numpy.prod(calib_bulb)))
        
# Complete calibration by computing the mean of readings for each of the sensors
sens_calib = sens_calib / no_passes

# =============================================================
# =============================================================
# FOR TESTING PURPOSES W. ARDUINO, SET CALIBRATED VALUES TO 100
# sens_calib = 100 * numpy.ones(no_sensors)

# =============================================================
# =============================================================

print("Calibration Complete!")
print("Calibration Values: ", sens_calib)

# ADD DATA TO BUFFER
while True:
    # Read, decode, strip serial message
    message = copy.deepcopy(clo_board.read_until())
    decoded_message = message.decode().strip()

    # Add label to buffer 
    serial_buffer.append(decoded_message[0:2])
    print(serial_buffer[-1])
    
    
    # If a sensor is triggered, also add its normalized value to the buffer
    if (serial_buffer[-1][0] != 'b'):
        
        if   (serial_buffer[-1][0:2] == 'f1'):
            # Append normalized flex sensor #1 value
            serial_buffer.append(numpy.clip(1 - float(decoded_message[3:]) / sens_calib[0],0,1))
            
            
        elif (serial_buffer[-1][0:2] == 'f2'):
            # Append normalized flex sensor #2 value
            serial_buffer.append(numpy.clip(1 - float(decoded_message[3:]) / sens_calib[1],0,1))
            
        elif (decoded_message[0:2] == 'Ax'):
            # Append normalized accelerometer 1 z-axis value
            #print("RAW: ", float(decoded_message[3:]))

            serial_buffer.append(abs(float(decoded_message[3:]) / sens_calib[2]))
            #print(serial_buffer[-1])
            
        elif (decoded_message[0:2] == 'Ay'):
            # Append normalized accelerometer 1 z-axis value
            #print("RAW: ", float(decoded_message[3:]))

            serial_buffer.append(abs(float(decoded_message[3:]) / sens_calib[2]))
            
        print(serial_buffer[-1])
            
       
                
                