from pyo import *
s = Server().boot()
s.start()
a = Sine(mul=0.01).out()