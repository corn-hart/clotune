#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 15 14:30:22 2022

@author: Radu
"""
import time
import pyo
# Creates and boots the server.
# The user should send the "start" command from the GUI.
s = pyo.Server(duplex=0)
s.setOutputDevice(1)
s.boot()
s.start()

# Rich frequency spectrum as stereo input source.
#amp = pyo.Fader(fadein=0.25, mul=0.5).play()
src = pyo.SfPlayer('/Users/Radu/Downloads/pluma.aiff')

# Flanger parameters                        == unit ==
middelay = 0.005  # seconds

depth = pyo.Sig(0.99)  # 0 --> 1
#depth.ctrl(title="Modulation Depth")
lfospeed = pyo.Sig(0.2)  # Hertz
#lfospeed.ctrl(title="LFO Frequency in Hz")
feedback = pyo.Sig(0.5, mul=0.95)  # 0 --> 1
#feedback.ctrl(title="Feedback")

# LFO with adjusted output range to control the delay time in seconds.
lfo = pyo.Sine(freq=lfospeed, mul=middelay * depth, add=middelay)

# Dynamically delayed signal. The source passes through a DCBlock
# to ensure there is no DC offset in the signal (with feedback, DC
# offset can be fatal!).
flg = pyo.Delay(pyo.DCBlock(src), delay=lfo, feedback=feedback)

# Mix the original source with its delayed version.
# Compress the mix to normalize the output signal.
# cmp = pyo.Compress(src + flg, thresh=-20, ratio=4).out()


t = pyo.CosTable([(0,0), (100,1), (500,.3), (8191,0)])
beat = pyo.Beat(time=.125, taps=16, w1=[90,80], w2=50, w3=35, poly=1).play()
trmid = pyo.TrigXnoiseMidi(beat, dist=12, mrange=(60, 96))
trhz = pyo.Snap(trmid, choice=[0,2,3,5,7,8,10], scale=1)
tr2 = pyo.TrigEnv(beat, table=t, dur=beat['dur'], mul=beat['amp'])
a = pyo.Sine(freq=trhz, mul=tr2*0.3).out()



try:
    while True:
        mode = float(input('Input:'))
        winsizee = mode
        #winsizee.setValue(mode) 
        print("Desired value: ", mode)
        time.sleep(0.01)
except:
    pass



