#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  1 13:39:24 2022

@author: Radu
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  1 12:44:56 2022

@author: Radu
"""

import pyo
import random

# =============================================================================
# =============================== START SERVER ================================
# =============================================================================

s = pyo.Server(duplex=0)
s.setOutputDevice(1)
s.boot()
s.start()

# =============================================================================
# ============================ INITIALIZE SOURCE ==============================
# =============================================================================

# Rich frequency spectrum as stereo input source.
#ap = pyo.Fader(fadein=0.25, mul=0.5).play()

t = pyo.CosTable([(0,0), (100,1), (500,.3), (8191,0)])
beat = pyo.Beat(time=.125, taps=16, w1=[90,80], w2=50, w3=35, poly=1).play()
trmid = pyo.TrigXnoiseMidi(beat, dist=12, mrange=(60, 96))
trhz = pyo.Snap(trmid, choice=[0,2,3,5,7,8,10], scale=1)
tr2 = pyo.TrigEnv(beat, table=t, dur=beat['dur'], mul=beat['amp'])
src = pyo.Sine(freq=trhz, mul=tr2*0.3*1).out()

"""
sf = pyo.SfPlayer('/Users/Radu/Downloads/kick.aiff')

gat = pyo.Gate(sf, thresh=-50, risetime=0.005, falltime=0.04, lookahead=4, outputAmp=True)
rev = pyo.STRev(sf, inpos=0.5, revtime=1.5, cutoff=5000, bal=1, roomSize=.8, firstRefGain=-3)
hip = pyo.ButHP(rev, freq=100)
cmp = pyo.Compress(hip, thresh=-12, ratio=3, risetime=0.005, falltime=0.05, lookahead=4, knee=0.5, mul=gat)

src = pyo.Interp(sf, cmp, interp=0.2, mul=0.5)
"""



# =============================================================================
# =================================== HPF =====================================
# =============================================================================

HPF_cutoff = pyo.Sig(0)
HPF_cutoff.ctrl(title="HPF")
HPF_sound = pyo.Biquad(src, freq = HPF_cutoff, type = 1)

# =============================================================================
# =================================== LPF =====================================
# =============================================================================
LPF_cutoff = pyo.Sig(10**6)
LPF_cutoff.ctrl(title="LPF")
LPF_sound = pyo.Biquad(HPF_sound, freq = LPF_cutoff, type = 0)

# =============================================================================
# ================================= FLANGER ===================================
# =============================================================================

# Flanger parameters
middelay = 0.005  # seconds

depth = pyo.Sig(0.99)  # 0 --> 1
depth.ctrl(title="Modulation Depth")
lfospeed = pyo.Sig(0.2)  # Hertz
lfospeed.ctrl(title="LFO Frequency in Hz")
feedback = pyo.Sig(0.5, mul=0.95)  # 0 --> 1
feedback.ctrl(title="Feedback")

# LFO with adjusted output range to control the delay time in seconds.
lfo = pyo.Sine(freq=lfospeed, mul=middelay * depth, add=middelay)

# Dynamically delayed signal. The source passes through a DCBlock
# to ensure there is no DC offset in the signal (with feedback, DC
# offset can be fatal!).
flg = pyo.Delay(pyo.DCBlock(src), delay=lfo, feedback=feedback)

# Mix the original source with its delayed version.
# Compress the mix to normalize the output signal.
flanger_sound = pyo.Compress(src + flg, thresh=-20, ratio=4)

# =============================================================================
# =============================== PING PONG ===================================
# =============================================================================

# Compute the duration, in seconds, of one buffer size.
buftime = s.getBufferSize() / s.getSamplingRate()

# Delay parameters
delay_time_l = pyo.Sig(0.125)  # Delay time for the left channel delay.
delay_time_l.ctrl(title = 'Delay_Time_L')
delay_feed = pyo.Sig(0.75)  # Feedback value for both delays.
delay_feed.ctrl(title = 'Delay_Feedback')

# Because the right delay gets its input sound from the left delay, while
# it is computed before (to send its output sound to the left delay), it
# will be one buffer size late. To compensate this additional delay on the
# right, we substract one buffer size from the real delay time.
delay_time_r = pyo.Sig(delay_time_l, add=-buftime)

# Send the original sound to both speakers.
ping_pong_sound = flanger_sound.mix(2).out()

# Initialize the right delay with zeros as input because the left delay
# does not exist yet.
right = pyo.Delay(pyo.Sig(0), delay=delay_time_r).out(1)

# Initialize the left delay with the original mono source and the right
# delay signal (multiplied by the feedback value) as input.
left = pyo.Delay(flanger_sound + right * delay_feed, delay=delay_time_l).out()

# One issue with recursive cross-delay is if we set the feedback to
# 0, the right delay never gets any signal. To resolve this, we add a
# non-recursive delay, with a gain that is the inverse of the feedback,
# to the right delay input.
original_delayed = pyo.Delay(flanger_sound, delay_time_l, mul=1 - delay_feed)

# Change the right delay input (now that the left delay exists).
right.setInput(original_delayed + left * delay_feed)


flanger_sound.play()

s.gui(locals())