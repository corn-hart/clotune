#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 15 14:30:22 2022

@author: Radu
"""
import time
import pyo
import serial
import time
import copy
import threading

class board_control():
    """
    This class is used to communicate with the controller. It is designed such that
    it can enable multiple controllers to be used at the same time for an even sicker
    musical experience
    
    """

    def __init__(self, audio_channel1, audio_channel2,  port_code, baud_r):
        """
        Initialize controller and associated audio channel/s

        """
        # Initiate communications over selected port
        self.comm_channel = serial.Serial(port = port_code, baudrate = baud_r)
        
        # Initialize button statuses
        self.Button1_status = -1
        self.Button2_status = -1
        
        # Declare audio channels controlled by this unit
        self.ch1 = audio_channel1
        self.ch2 = audio_channel2

    
    def parse(self):
        """
        Access the FIFO serial buffer and depending on the requested label,
        send the proper commands to the sound and visual GUI systems

        """
        # Access serial command buffer
        global serial_buffer
        
        # Wait until buffer holds values
        while (len(serial_buffer)<2):
            time.sleep(0.01)
            
        
        # BUTTON 1 ACTUATION
        if (serial_buffer[0] == 'b1'):
            # Remove label from buffer
            del serial_buffer[0]
            # Send command to sound system
            self.Button1()
            
            # INSERT VISUAL COMPONENT FOR BUTTON 1 HERE
            
        # BUTTON 2 ACTUATION
        elif (serial_buffer[0] == 'b2'):
            # Remove label from buffer
            del serial_buffer[0]
            # Send command to sound system
            self.Button2()
            
            # INSERT VISUAL COMPONENT FOR BUTTON 2 HERE
        
        # FLEX 1 INPUT
        elif (serial_buffer[0] == 'f1'):
            # Copy the requested control input from the buffer
            control_input = copy.deepcopy(serial_buffer[1])
            # Remove label and command from buffer
            del serial_buffer[0:2]
            # Send command to sound system
            self.Flex1(control_input)
            
            # INSERT VISUAL COMPONENT FOR FLEX 1 HERE

        # FLEX 2 INPUT
        elif (serial_buffer[0] == 'f2'):
            # Copy the requested control input from the buffer
            control_input = copy.deepcopy(serial_buffer[1])
            # Remove label and command from buffer
            del serial_buffer[0:2]
            # Send command to sound system
            self.Flex2(control_input)
            
            # INSERT VISUAL COMPONENT FOR FLEX 2 HERE
                
    def Button1(self):
        """
        Toggle status of Button1 and enable/disable associated effect

        """
        # Toggle memory item and enable/disable effect1 (LPF) on this channel
        self.Button1_status = - self.Button1_status
        self.ch1.effect_1()
        
    def Button2(self):
        """
        Toggle status of Button2 and enable/disable associated effect

        """
        # Toggle memory item and enable/disable effect1 (LPF) on this channel
        self.Button2_status = - self.Button2_status
        self.ch1.effect_2()
        
    def Flex1(self, command_input):
        """
        Apply Flex sensor 1 input
        """
        self.ch1.effect_3(command_input)
       
    def Flex2(self, command_input):
        """
        Apply Flex sensor 2 input
        """
        self.ch1.effect_4(command_input)
        
class audio_channel():
    
    """
    This class is used to control audio channels with inputs. Again, the reason
    this is designed as a class is such that it can be used to initiate mutliple
    concomittent audio channels for an even richer musical experience
    """

    def __init__(self, server, code, file_address= 'placeholder'):
        """
        INITIALIZE AUDIO SOURCE AND ITS EFFECTS
        code: 0 to initialize synthetic sound 0
        code: 1 to read from sound file

        """
        self.server = server
        
        if (code == 0):
            self.source = pyo.Sine(freq=trhz, mul=tr2*0.3)
        elif (code ==1):  
            self.source = pyo.SfPlayer(file_address)
        else:
            pass
        
        # ====================================================================
        # ========================= EFFECT 1 (HPF) ===========================
        # ====================================================================

        # CONTROLS: 1 (ON/OFF)
        self.HPF_control = -1
        self.HPF_cutoff = pyo.Sig(0)
        initial_filtered_sound = pyo.Biquad(self.source, freq = self.HPF_cutoff, type = 1)

        
        # ====================================================================
        # ========================= EFFECT 2 (LPF) ===========================
        # ====================================================================

        # CONTROLS: 1 (ON/OFF)
        self.LPF_control = -1
        self.LPF_cutoff = pyo.Sig(10**6)
        filtered_sound = pyo.Biquad(initial_filtered_sound, freq = self.LPF_cutoff, type = 0)

        
        # ====================================================================
        # ====================== EFFECT 3 (FLANGER) ==========================
        # ====================================================================

        # CONTROLS: 1 (Feedback)
        
        middelay = 0.005  # Seconds
        depth = pyo.Sig(0.5)  # (Modulation Depth) 0 --> 1
        lfospeed = pyo.Sig(0.2)  # LFO Frequency HZ
        self.feedback = pyo.Sig(0, mul=0.95)  # Feedback 0 --> 1

        # LFO with adjusted output range to control the delay time in seconds.
        lfo = pyo.Sine(freq=lfospeed, mul=middelay * depth, add=middelay)

        # Dynamically delayed signal. The source passes through a DCBlock
        # to ensure there is no DC offset in the signal (with feedback, DC
        # offset can be fatal!).
        delayed = pyo.Delay(pyo.DCBlock(filtered_sound), delay=lfo, feedback=self.feedback)

        # Mix the original source with its delayed version.
        # Compress the mix to normalize the output signal.
        thru_sound = pyo.Compress(filtered_sound + delayed, thresh=-20, ratio=4)
           
        
        # ====================================================================
        # ===================== EFFECT 4 (PING-PONG) =========================
        # ====================================================================

        # CONTROLS: 2 (Delay_time, Delay_feedback)
        
        # Compute the duration, in seconds, of one buffer size.
        buftime = self.server.getBufferSize() / self.server.getSamplingRate()
        
        # Delay parameters
        self.delay_time_l = pyo.Sig(0.125)  # Delay time for the left channel delay.
        #delay_time_l.ctrl()
        self.delay_feed = pyo.Sig(0.75)  # Feedback value for both delays.
        #delay_feed.ctrl()
        
        # Because the right delay gets its input sound from the left delay, while
        # it is computed before (to send its output sound to the left delay), it
        # will be one buffer size late. To compensate this additional delay on the
        # right, we substract one buffer size from the real delay time.
        self.delay_time_r = pyo.Sig(self.delay_time_l, add=-buftime)
        
        # Send the output of the flanger to both speakers
        sfout = thru_sound.mix(2).out()
        
        # Initialize the right delay with zeros as input because the left delay
        # does not exist yet.
        right = pyo.Delay(pyo.Sig(0), delay=self.delay_time_r).out(1)

        # Initialize the left delay with the original mono source and the right
        # delay signal (multiplied by the feedback value) as input.
        left = pyo.Delay(thru_sound + right * self.delay_feed, delay=self.delay_time_l).out()

        # One issue with recursive cross-delay is if we set the feedback to
        # 0, the right delay never gets any signal. To resolve this, we add a
        # non-recursive delay, with a gain that is the inverse of the feedback,
        # to the right delay input.
        original_delayed = pyo.Delay(thru_sound, self.delay_time_l, mul=1 - self.delay_feed)

        # Change the right delay input (now that the left delay exists).
        right.setInput(original_delayed + left * self.delay_feed)
        
        # Play the sound after the ping_pong effect has been applied
        thru_sound.play()

    def effect_1(self):
        """
        HPF

        """
        # Toggle HPF control
        self.HPF_control = - self.HPF_control
        
        # Set appropriate cutoff frequency
        if (self.HPF_control == -1):
            self.HPF_cutoff.setValue(0) 
            
        else:
            self.HPF_cutoff.setValue(1000) 

    def effect_2(self):
        """
        LPF

        """
        # Toggle LPF control
        self.LPF_control = - self.LPF_control
        
        # Set appropriate cutoff frequency
        if (self.LPF_control == -1):
            self.LPF_cutoff.setValue(10**6) 
            
        else:
            self.LPF_cutoff.setValue(400) 
        
    def effect_3(self, input_control):
        """
        FLANGER

        """
        # Apply new flanger feedback
        self.feedback.setValue(input_control)
        
    def effect_4(self, input_control):
        """
        PING-PONG

        """
        # Apply new ping_pong delay
        self.delay_time_l.setValue(input_control)

# Initialize and boot pyo server
pyo_serv = pyo.Server(duplex=0)
pyo_serv.setOutputDevice(1)
pyo_serv.boot()
pyo_serv.start()

"""
DECLARED GLOBAL VARIABLES FOR USE
"""

serial_buffer = []

# Initialize first audio channel
# track_1 = audio_channel(pyo_serv, 0, '/Users/Radu/Downloads/pluma.aiff')
track_2 = audio_channel(pyo_serv, 0)

# Initialize second audio channel
time.sleep (0.01)
track_1 = audio_channel(pyo_serv, 1, '/Users/Radu/Downloads/kick.aiff')

    
# Intialize clotune board
clo_board = board_control(track_1, track_2, '/dev/tty.usbmodem14201', 9600)

    
def add_to_buffer():
    global serial_buffer, clo_board
    
    # Store calibration values for all sensors
    f1_calib = 0.0
    f2_calib = 0.0
    
    # SENSOR CALIBRATION
    while (min(f1_calib,f2_calib)==0.0):
        # Read, decode, strip serial message
        message = copy.deepcopy(clo_board.comm_channel.read_until())
        decoded_message = message.decode().strip()

        # If a sensor is triggered, calibrate that sensor
        if (decoded_message[0] != 'b'):
            
            if (decoded_message[0:2] == 'f1'):
                # Calibrate flex sensor #1
                f1_calib = int(decoded_message[3:])
                
            elif (decoded_message[0:2] == 'f2'):
                # Calibrate flex sensor #2
                f2_calib = int(decoded_message[3:]) 
                
        # If a button is triggered, add it to the buffer
        else:
            serial_buffer.append(decoded_message[0:2])

    # =============================================================
    # =============================================================
    # FOR TESTING PURPOSES W. ARDUINO, SET CALIBRATED VALUES TO 100
    f1_calib = 100.0
    f2_calib = 100.0
    # =============================================================
    # =============================================================

    
    # ADD DATA TO BUFFER
    while True:
        # Read, decode, strip serial message
        message = copy.deepcopy(clo_board.comm_channel.read_until())
        decoded_message = message.decode().strip()
    
        # Add label to buffer
        serial_buffer.append(decoded_message[0:2])
        
        # If a sensor is triggered, also add its normalized value to the buffer
        if (serial_buffer[-1][0] != 'b'):
            
            if   (serial_buffer[-1][0:2] == 'f1'):
                # Append normalized flex sensor #1 value
                serial_buffer.append(f1_calib - int(decoded_message[3:]))
                
            elif (serial_buffer[-1][0:2] == 'f2'):
                # Append normalized flex sensor #2 value
                serial_buffer.append(f2_calib - int(decoded_message[3:]))
               
def clotune():
    global clo_board
 
    while True:
        clo_board.parse()
        
          
def main():
    global clo_board, track_1, pyo_serv

    # Create Multithreaded Recieve - Affect Process
    serial_thread = threading.Thread(target=add_to_buffer)
    audio_thread = threading.Thread(target=clotune)
    serial_thread.start()
    audio_thread.start()  
    serial_thread.join()
    audio_thread.join()
    clo_board.comm_channel.close()
    pyo_serv.stop()




"""
:) EXECUTE CLOTUNE :)
"""
if (__name__ == "__main__"):
    main()       
else:
    pass
        



