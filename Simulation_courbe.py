#!/usr/bin/env python3

import matplotlib.pyplot as plt
from math import sqrt


'''

Fichier de simulation de courbe de vitesse et de position.
Il permet de comprendre plus facilement le fonctionnement 
des courbes d'accélération et de décélération, en affichant
les courbes. Si la vitesse max est ateingnable, on obtient
une courbe en trapèze. Sinon, on obtient une courbe en triangle.

'''

Amax = 1000 # Accélération constante
Vmax = 500  # Vitesse max
d0 = -1000 # Position de départ
d = 0 # Position d'arrivée

t1 = Vmax/Amax # Temps d'accélération
t2 = ((d-d0)-Amax*t1*t1)/Vmax + t1 # Temps de commencement de la décélération


dLim = Vmax*Vmax/Amax # distance mini pour atteindre la vitesse max

# Si la distance est trop petite pour atteindre la vitesse max
# Il n'y a plus de trapèze mais un triangle
if(abs(d-d0) < abs(dLim)):
    t1 = sqrt((d-d0)/Amax)
    t2 = t1 
    Vmax = Amax*t1

tf = t1 + t2 # Temps total

outT = [] # Temps
outV = [] # Vitesse
outP = [] # Position

# Calcul des valeurs pour chaque instant
# On considère un pas de 10ms
for tic in range (0,400):
    t = tic*10/1000 # Temps en secondes
    outT.append(t) # Ajout du temps

    # Calcul de la vitesse et de la position

    #Phase d'accélération
    if t < t1:
        v = Amax*t
        p = Amax*t*t/2 + d0
    
    #Phase d'arrêt
    elif t > tf:
        v = 0
        p = Amax*t1*t1/2 + Vmax*(t2) -Amax*(t1)*(t1)/2 + d0

    #Phase de décélération
    elif t > t2:
        v = Vmax - Amax*(t-t2)
        p = Amax*t1*t1/2 + Vmax*(t-t1) -Amax*(t-t2)*(t-t2)/2 + d0
    
    #Phase de vitesse constante
    else:
        v = Vmax
        p = Amax*t1*t1/2 + Vmax*(t-t1) + d0
    outV.append(v) # Ajout de la vitesse
    outP.append(p) # Ajout de la position


plt.plot(outT, outV)
plt.plot(outT, outP)
plt.show()
