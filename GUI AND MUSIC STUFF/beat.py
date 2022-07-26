#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  2 22:31:03 2022

@author: Radu
"""

import time
import pyo
import numpy
a = numpy.array([0,1,2])

# Creates and boots the server.
# The user should send the "start" command from the GUI.

s = pyo.Server(duplex=0)
s.setOutputDevice(1)
s.boot()
s.start()
"""
t = pyo.CosTable([(0,0), (100,1), (500,.3), (8191,0)])
beat = pyo.Beat(time=.125, taps=16, w1=[90,80], w2=50, w3=35, poly=1).play()
trmid = pyo.TrigXnoiseMidi(beat, dist=12, mrange=(60, 96))
trhz = pyo.Snap(trmid, choice=[0,2,3,5,7,8,10], scale=1)
tr2 = pyo.TrigEnv(beat, table=t, dur=beat['dur'], mul=beat['amp'])
src = pyo.Sine(freq=trhz, mul=tr2*0.3)
"""

src = pyo.SfPlayer('/Users/Radu/Downloads/sun.aiff',speed = 0.9833).play(delay = 0.12)

sf = pyo.SfPlayer('/Users/Radu/Downloads/kick.aiff').play(delay = 0.25)

gat = pyo.Gate(sf, thresh=-50, risetime=0.005, falltime=0.04, lookahead=4, outputAmp=True)
rev = pyo.STRev(sf, inpos=0.5, revtime=1.5, cutoff=5000, bal=1, roomSize=.8, firstRefGain=-3)
hip = pyo.ButHP(rev, freq=100)
cmp = pyo.Compress(hip, thresh=-12, ratio=3, risetime=0.005, falltime=0.05, lookahead=4, knee=0.5, mul=gat)

src2 = pyo.Interp(sf, cmp, interp=0.2, mul=0.5)

sound_out = pyo.Compress(src + sf, thresh=-20, ratio=4).out()





