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
import pygame
import numpy
import sys

class board_control():
    """
    This class is used to communicate with the controller. It is designed such that
    it can enable multiple controllers to be used at the same time for an even sicker
    musical experience
    
    """

    def __init__(self, audio_channel1, audio_channel2, audio_channel3, port_code, baud_r):
        """
        Initialize controller and associated audio channel/s

        """
        # Initiate communications over selected port
        self.comm_channel = serial.Serial(port = port_code, baudrate = baud_r)
        
        # Declare audio channels controlled by this unit
        self.ch1 = audio_channel1 # KICKDRUM
        self.ch2 = audio_channel2 # SYNTHETIC
        self.ch3 = audio_channel3 # SUN
        
        # Reconfigure default values for some track effects
        self.ch1.effect_4(0) # Ping Pong Delay Time L off for KICKDRUM
        self.ch1.effect_5(0) # Ping Pong Delay FEEDBACK off for KICKDRUM
        
        self.ch2.effect_3(0) # Flanger off for SYNTHETIC
        
        self.ch3.effect_4(0) # Ping Pong Delay Time L off for SUN
        self.ch3.effect_5(0) # Ping Pong Delay FEEDBACK off for SUN

        
    def parse(self):
        """
        Access the FIFO serial buffer and depending on the requested label,
        send the proper commands to the sound and visual GUI systems

        """
        # Access serial command buffer
        global serial_buffer, GUI_ins
        
        # Wait until buffer holds values
        if (len(serial_buffer)>2):
            # print('LEN: ',len(serial_buffer))
            # print('SGNL: ',serial_buffer[0])
            
            # print("EXECUTING CMD: ", serial_buffer[0:2])
            
            # BUTTON 1 ACTUATION
            if (serial_buffer[0] == 'b1'):
                # Remove label from buffer
                del serial_buffer[0]
                # Send command to sound system
                self.Button1()
                
                # Update GUI object
                GUI_ins[0] = - GUI_ins[0]
                
                
            # BUTTON 2 ACTUATION
            elif (serial_buffer[0] == 'b2'):
                # Remove label from buffer
                del serial_buffer[0]
                # Send command to sound system
                self.Button2()
                
                # Update GUI object
                GUI_ins[1] = - GUI_ins[1]
                
            # BUTTON 3 ACTUATION
            elif (serial_buffer[0] == 'b3'):
                # Remove label from buffer
                del serial_buffer[0]
                # Send command to sound system
                self.Button3()
                
                # Update GUI object
                GUI_ins[2] = - GUI_ins[2]
            
            # FLEX 1 INPUT
            elif (serial_buffer[0] == 'f1'):
                # Copy the requested control input from the buffer
                control_input = copy.deepcopy(serial_buffer[1])
                # Remove label and command from buffer
                del serial_buffer[0:2]
                # Send command to sound system
                self.Flex1(float(control_input))
                
                # Update GUI object
                GUI_ins[3] = int(150*(control_input))+350
    
            # FLEX 2 INPUT
            elif (serial_buffer[0] == 'f2'):
                # Copy the requested control input from the buffer
                control_input = copy.deepcopy(serial_buffer[1])
                # Remove label and command from buffer
                del serial_buffer[0:2]
                # Send command to sound system
                self.Flex2(float(control_input))
                
                # Update GUI object
                GUI_ins[4] = int(150*(control_input))+350
                
            # ACC 1 X INPUT
            elif (serial_buffer[0] == 'Ax'):
                # Copy the requested control input from the buffer
                control_input = copy.deepcopy(serial_buffer[1])
                # Remove label and command from buffer
                del serial_buffer[0:2]
                # Send command to sound system
                self.Acc_x_1(float(control_input))
                
                # Update GUI object
                GUI_ins[5] = int(150*(control_input))+100
                
            # ACC 1 Y INPUT
            elif (serial_buffer[0] == 'Ay'):
                # Copy the requested control input from the buffer
                control_input = copy.deepcopy(serial_buffer[1])
                # Remove label and command from buffer
                del serial_buffer[0:2]
                # Send command to sound system
                self.Acc_y_1(float(control_input))
                
                # Update GUI object
                GUI_ins[6] = int(150*(control_input))+100
            
        else:
            time.sleep(0.1)
            #print("WAITING ON BUFFER")
                
    def Button1(self):
        """
        Toggle status of Button1 and enable/disable track

        """
        self.ch1.set_on_off()
        
    def Button2(self):
        """
        Toggle status of Button2 and enable/disable track

        """
        self.ch2.set_on_off()
        
    def Button3(self):
        """
        Toggle status of Button3 and enable/disable track

        """
        self.ch3.set_on_off()      
    
    def Flex1(self, command_input):
        """
        Apply Flex sensor 1 input
        """
        self.ch2.effect_4(command_input)
       
    def Flex2(self, command_input):
        """
        Apply Flex sensor 2 input
        """
        self.ch2.effect_5(command_input)
        
    def Acc_x_1(self, command_input):
        """
        Apply Accel_1_x sensor input
        """
        self.ch1.effect_3(command_input)          

    def Acc_y_1(self, command_input):
        """
        Apply Accel_1_y sensor input
        """
        self.ch3.effect_3(command_input)          
      
class audio_channel():
    
    """
    This class is used to control audio channels with inputs. Again, the reason
    this is designed as a class is such that it can be used to initiate mutliple
    concomittent audio channels for an even richer musical experience
    """

    def __init__(self, server, code, file_address= 'placeholder'):
        """
        INITIALIZE AUDIO SOURCE AND ITS EFFECTS
        code: 0 to initialize hats
        code: 1 to initialize kickdrum
        code: 2 to initialize who "loves the sun"

        """
        self.server = server
        self.on_off = pyo.Sig(value = 0)
        
        if (code == 0):
            sf = pyo.SfPlayer(file_address, mul = self.on_off).play(delay = 0.25)
            gat = pyo.Gate(sf, thresh=-50, risetime=0.005, falltime=0.04, lookahead=4, outputAmp=True)
            rev = pyo.STRev(sf, inpos=0.5, revtime=1.5, cutoff=5000, bal=1, roomSize=.8, firstRefGain=-3)
            hip = pyo.ButHP(rev, freq=100)
            cmp = pyo.Compress(hip, thresh=-12, ratio=3, risetime=0.005, falltime=0.05, lookahead=4, knee=0.5, mul=gat)
            self.source = pyo.Interp(sf, cmp, interp=0.2, mul=0.5)
        elif (code == 1):  
            self.source = pyo.SfPlayer(file_address, mul = self.on_off).play(delay = 0.25) # SET DELAY FOR BEATMATCHING HERE
        elif (code == 2):
            self.source = pyo.SfPlayer(file_address, speed = 0.9833, mul = self.on_off).play(delay = 0.12) 
        else:
            pass
        
        # ====================================================================
        # ============================ ON / OFF ==============================
        # ====================================================================
        
        # CONTROLS: 1 (ON/OFF) 
        self.on_off_control = -1
        self.on_off.setValue(0) 
        
        # ====================================================================
        # ========================= EFFECT 1 (HPF) ===========================
        # ====================================================================

        # CONTROLS: 1 (ON/OFF)
        self.HPF_cutoff = pyo.Sig(value = 0)
        initial_filtered_sound = pyo.Biquad(self.source, freq = self.HPF_cutoff, type = 1)

        
        # ====================================================================
        # ========================= EFFECT 2 (LPF) ===========================
        # ====================================================================

        # CONTROLS: 1 (ON/OFF)
        self.LPF_cutoff = pyo.Sig(value = 10**6)
        filtered_sound = pyo.Biquad(initial_filtered_sound, freq = self.LPF_cutoff, type = 0)

        
        # ====================================================================
        # ====================== EFFECT 3 (FLANGER) ==========================
        # ====================================================================

        # CONTROLS: 1 (Feedback)
        
        middelay = 0.005  # Seconds
        self.depth = pyo.Sig(value = 0.1)  # (Modulation Depth) 0 --> 1
        self.lfospeed = pyo.Sig(value = 0.1)  # LFO Frequency HZ
        self.feedback = pyo.Sig(value = 0.1, mul=0.95)  # Feedback 0 --> 1

        # LFO with adjusted output range to control the delay time in seconds.
        lfo = pyo.Sine(freq=self.lfospeed, mul=middelay * self.depth, add=middelay)

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
        self.delay_time_l = pyo.Sig(value = 0.1)  # Delay time for the left channel delay.
        #delay_time_l.ctrl()
        self.delay_feed = pyo.Sig(value = 0.1)  # Feedback value for both delays.
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
        # thru_sound.play()

    def set_on_off(self):
        """
        ON/OFF

        """
        # Toggle ON/OFF control
        self.on_off_control = - self.on_off_control
        
        # Set appropriate cutoff frequency
        if (self.on_off_control == -1):
            self.on_off.setValue(0) 
        else:
            self.on_off.setValue(1) 

        
    def effect_1(self, input_control):
        """
        HPF

        """
        self.HPF_cutoff.setValue(input_control * 500) 

    def effect_2(self, input_control):
        """
        LPF

        """
        self.LPF_cutoff.setValue(input_control * 80) 

        
    def effect_3(self, input_control):
        """
        FLANGER

        """
        # Apply new flanger feedback
        self.feedback.setValue(input_control)
        self.lfospeed.setValue(input_control)
        self.depth.setValue(input_control)
        
        
    def effect_4(self, input_control):
        """
        PING-PONG DELAY TIME L

        """
        # Apply new ping_pong delay
        self.delay_time_l.setValue(input_control/4)
        
    def effect_5(self, input_control):
        """
        PING-PONG DELAY FEEDBACK

        """
        # Apply new ping_pong delay
        self.delay_feed.setValue(input_control/4)

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
track_1 = audio_channel(pyo_serv, 1, '/Users/Radu/Downloads/kick.aiff')

# Initialize second audio channel
track_2 = audio_channel(pyo_serv, 0, '/Users/Radu/Downloads/hats.aiff')

# Initialize third audio channel
track_3 = audio_channel(pyo_serv, 2, '/Users/Radu/Downloads/sun.aiff')

# Intialize clotune board
clo_board = board_control(track_1, track_2, track_3, '/dev/tty.usbmodem14203', 115200)

# Interface states for use by GUI
GUI_ins = numpy.ones(7)


def add_to_buffer():
    global serial_buffer, clo_board
    
    # Store calibration values for all sensors
    no_sensors = 4
    # Dictates the number of readings needed for each sensor to complete its calibration
    no_passes = 4
    sens_calib = numpy.zeros(no_sensors)
    calib_bulb = numpy.zeros(no_sensors)
    sens_labels = ['f1', 'f2', 'Ax', 'Ay']

    # SENSOR CALIBRATION
    while (int(numpy.prod(calib_bulb)) != no_passes ** no_sensors):
        # Read, decode, strip serial message
        message = copy.deepcopy(clo_board.comm_channel.read_until())
        decoded_message = message.decode().strip()

        # For non-empty messages
        if decoded_message:
            
            # If a sensor is triggered, calibrate that sensor
            if (decoded_message[0] != 'b'):
                
                # Save the reference value for each sensor
                for jj in range (no_sensors):
                    # If a certain sensor sends data, record its value
                    if (decoded_message[0:2] == sens_labels[jj]) and (calib_bulb[jj] < no_passes):
                        # Add the read value to the averaging function
                        sens_calib[jj] += int(decoded_message[3:])
                        # Calibrate each sensor ''no_passes'' times
                        calib_bulb[jj] += 1
                        break
                
            # Print status
            print("Calibration: ",sum(calib_bulb),'/',no_sensors*no_passes)
                
                
    # Complete calibration by computing the mean of readings for each of the sensors
    sens_calib = sens_calib / no_passes
    
    # =============================================================
    # =============================================================
    # FOR TESTING PURPOSES W. ARDUINO, SET CALIBRATED VALUES TO 100
    # sens_calib = 100 * numpy.ones(no_sensors)

    # =============================================================
    # =============================================================

    print("Calibration Complete!")
    print("BUFFER: ", serial_buffer)
    
    # ADD DATA TO BUFFER
    while True:
        # Read, decode, strip serial message
        message = copy.deepcopy(clo_board.comm_channel.read_until())
        decoded_message = message.decode().strip()
    
        # For non-empty messages
        if decoded_message:
            
            # Append label
            serial_buffer.append(decoded_message[0:2])
            
                    
            # If a sensor is triggered, also add its normalized value to the buffer
            if (serial_buffer[-1][0] != 'b'):
                
                if   (serial_buffer[-1][0:2] == 'f1'):
                    # Append normalized flex sensor #1 value
                    serial_buffer.append(numpy.clip(1 - float(decoded_message[3:]) / sens_calib[0],0,1))
                    
                elif (serial_buffer[-1][0:2] == 'f2'):
                    # Append normalized flex sensor #2 value
                    serial_buffer.append(numpy.clip(1 - float(decoded_message[3:]) / sens_calib[1],0,1))

                elif (decoded_message[0:2] == 'Ax'):
                    # Append normalized accelerometer 2 z-axis value
                    serial_buffer.append(numpy.clip(abs(float(decoded_message[3:])) / sens_calib[2],0,1))
                    
                elif (decoded_message[0:2] == 'Ay'):
                    # Append normalized accelerometer 2 z-axis value
                    serial_buffer.append(numpy.clip(abs(float(decoded_message[3:])) / sens_calib[3],0,1))
                    
def clotune():
    global clo_board, GUI_ins
 
    while True:
        # AFFECT SOUND
        clo_board.parse()
          
def main():
    global clo_board, track_1, pyo_serv, GUI_ins

    # =========================================================================
    # ============================= MULTITHREAD ===============================
    # =========================================================================    
    
    serial_thread = threading.Thread(target=add_to_buffer)
    clotune_thread = threading.Thread(target=clotune)
    serial_thread.start()
    clotune_thread.start() 
    
    # =========================================================================
    # =========================== INITIALIZE GUI ==============================
    # =========================================================================
    
    # These are RGB colors that pygame uses, and we can name a couple of them for ease of use
    pygame.font.init()
    
    GREY = (70,70,70)
    WHITE = (255,255,255)

    # This is the value at which we will have the GUI update 
    FPS = 60
    SCREEN_WIDTH = 1024
    SCREEN_HEIGHT = 700

    # Initialize the window
    WIN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption('Clotune GUI')

    # The size od the images that we will use shortly 
    BUTTON_WIDTH = 70
    BUTTON_HEIGHT = 70
    FLEX_WIDTH = 50
    FLEX_HEIGHT = 50

    #Load in the images that we will use as the buttons and sliders
    BACKGROUND_IMAGE = pygame.image.load('BACKGROUND.jpeg')
    BUTTON1_ON_IMAGE = pygame.image.load('BUTTON_ON.png')
    BUTTON1_OFF_IMAGE = pygame.image.load('BUTTON_OFF.png')
    FLEX_IMAGE = pygame.image.load('FLEX.jpeg').convert_alpha()
    SLIDER_IMAGE = pygame.image.load('SLIDER.jpeg').convert_alpha()

    # Transform every lodaed image to the size that we want
    SLIDER = pygame.transform.scale(SLIDER_IMAGE, (50, 200))
    BACKGROUND = pygame.transform.scale(BACKGROUND_IMAGE, (SCREEN_WIDTH, SCREEN_HEIGHT))
    BUTTON1_ON = pygame.transform.scale(BUTTON1_ON_IMAGE, (BUTTON_WIDTH, BUTTON_HEIGHT))
    BUTTON1_OFF = pygame.transform.scale(BUTTON1_OFF_IMAGE, (BUTTON_WIDTH, BUTTON_HEIGHT))
    FLEX = pygame.transform.scale(FLEX_IMAGE, (FLEX_WIDTH, FLEX_HEIGHT))

    # Border for the slider
    BORDER = pygame.Rect(0, SCREEN_HEIGHT/2.5, 600, 700)
    slider1_border = pygame.Rect(700, 350, 50, 200) # flex sensor 1 
    slider2_border = pygame.Rect(800, 350, 50, 200) # flex sensor 2
    slider3_border = pygame.Rect(100, 350, 200, 50) # accel sensor 1
    slider4_border = pygame.Rect(100, 450, 200, 50) # accel sensor 2
    # slider5_border = pygame.Rect(100, 550, 200, 50) # who knows at this point

    #Get the font that we want for the Captions
    title_font = pygame.font.SysFont('americantypewriter', 150)
    button_font = pygame.font.SysFont('americantypewriter', 20)

    current_button1 = BUTTON1_ON
    current_button2 = BUTTON1_ON
    current_button3 = BUTTON1_ON
    flex1 = pygame.Rect(700, 450, FLEX_WIDTH, FLEX_HEIGHT)
    flex2 = pygame.Rect(800, 450, FLEX_WIDTH, FLEX_HEIGHT ) 
    accel1 = pygame.Rect(150, 350, FLEX_WIDTH, FLEX_HEIGHT ) 
    accel2 = pygame.Rect(150, 350, FLEX_WIDTH, FLEX_HEIGHT )
    # accel3 = pygame.Rect(150, 350, FLEX_WIDTH, FLEX_HEIGHT )
    clock = pygame.time.Clock()
    i = 0
        
    run = True
    
    # =========================================================================
    # ============================= UPDATE GUI ================================
    # =========================================================================
    
    time.sleep(3)
    
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False    
                
        if pygame.key.get_pressed()[pygame.K_ESCAPE]:
            run = False
                
        WIN.blit(BACKGROUND, (i, 0))
        WIN.blit(BACKGROUND, (SCREEN_WIDTH+i, 0))
        if i == -SCREEN_WIDTH:
            WIN.blit(BACKGROUND, (SCREEN_WIDTH+i, 0))
            i = 0
        i -= 1        
        
        #WIN.blit(BACKGROUND, (0,0)) # This fills the screen with white as the numbers indicate RGB, with a range of 0-255
        clotune_text = title_font.render('CLOTUNE', 300, GREY)
        button1_text = button_font.render('KICK', 500, GREY)
        button2_text = button_font.render('HATS', 500, GREY)
        button3_text = button_font.render('SUN', 500, GREY)
        flex1_text = button_font.render('PPONG_DELAY', 500, GREY)
        flex2_text = button_font.render('PPONG_FEED', 500, GREY)
        accelx_text = button_font.render('FLANGR__KICK', 500, GREY)
        accely_text = button_font.render('FLANGR__SUN', 500, GREY)
        WIN.blit(clotune_text, (150, 50))
        WIN.blit(button1_text, (500, 275))
        WIN.blit(button2_text, (500, 375))
        WIN.blit(button3_text, (500, 475))
        WIN.blit(flex1_text, (700, 325))
        WIN.blit(flex2_text, (800, 325))
        WIN.blit(accelx_text, (100, 325))
        WIN.blit(accely_text, (100, 425))
        pygame.draw.rect(WIN,GREY, slider1_border)
        pygame.draw.rect(WIN,GREY, slider2_border)
        pygame.draw.rect(WIN,GREY, slider3_border)
        pygame.draw.rect(WIN,GREY, slider4_border)
        # pygame.draw.rect(WIN,GREY, slider5_border)
        
        # Set proper images for all buttons
        if (GUI_ins[0] == -1):
            current_button1 = BUTTON1_OFF
        else: 
            current_button1 = BUTTON1_ON
            
        if (GUI_ins[1] == -1):
            current_button2 = BUTTON1_OFF
        else: 
            current_button2 = BUTTON1_ON
            
        if (GUI_ins[2] == -1):
            current_button3 = BUTTON1_OFF
        else: 
            current_button3 = BUTTON1_ON
                        
        # Update button images
        WIN.blit(current_button1, (500,300))
        WIN.blit(current_button2, (500,400))
        WIN.blit(current_button3, (500,500))
        
        # Update flex sensor positions
        WIN.blit(FLEX, (700, GUI_ins[3]))
        WIN.blit(FLEX, (800, GUI_ins[4]))
    
        # Update accelerometer sensor positions
        WIN.blit(FLEX, (GUI_ins[5], 350))
        WIN.blit(FLEX, (GUI_ins[6], 450))
        # WIN.blit(FLEX, (accel3.x, 550))
        pygame.display.update()
    
    
    serial_thread.join()
    clotune_thread.join()
    clo_board.comm_channel.close()
    pyo_serv.stop()


"""
:) EXECUTE CLOTUNE :)
"""
if (__name__ == "__main__"):
    main()       
else:
    pass
        



