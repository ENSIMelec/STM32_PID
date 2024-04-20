#!/usr/bin/env python3

import matplotlib.pyplot as plt
from math import sqrt

Amax = -1000
Vmax = -500
d0 = 100
d = 0

t1 = Vmax/Amax
t2 = ((d-d0)-Amax*t1*t1)/Vmax + t1


dLim = Vmax*Vmax/Amax


if(abs(d-d0) < abs(dLim)):
    t1 = sqrt((d-d0)/Amax)
    t2 = t1 
    Vmax = Amax*t1
tf = t1 + t2

outT = []
outV = []
outP = []
for tic in range (0,400):
    t = tic*10/1000
    outT.append(t)
    if t < t1:
        v = Amax*t
        p = Amax*t*t/2 + d0
    elif t > tf:
        v = 0
        p = d
    elif t > t2:
        v = Vmax - Amax*(t-t2)
        p = Amax*t1*t1/2 + Vmax*(t-t1) -Amax*(t-t2)*(t-t2)/2 + d0
    else:
        v = Vmax
        p = Amax*t1*t1/2 + Vmax*(t-t1) +d0
    outV.append(v)
    outP.append(p)

plt.plot(outT, outV)

plt.plot(outT, outP)
plt.show()
