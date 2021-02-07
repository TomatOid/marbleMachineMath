#!/usr/bin/env python # coding: utf-8

# In[ ]:

from math import *

g = 9.8
r = 1

def trackWidth(position):
    return 0.25 + 0.1 * position

def rollingRadius(position):
    return sqrt(r ** 2 - trackWidth(position) ** 2)

def pitchAngle(position, delta_t):
    return -atan((rollingRadius(position + delta_t) - rollingRadius(position)) / delta_t)

def frictionCoefficient(position, delta_t):
    return 0.1 * cos(pitchAngle(position, delta_t)) * sqrt(1 + (trackWidth(position) / rollingRadius(position)))

def ssqrt(x):
    return copysign(1, x) * sqrt(abs(x))

def noSlip(lin_vel, ang_vel, position, delta, r_roll, mu_k, creepage):
    denominator = r ** 2 / (5 * r_roll ** 2) + 1 / 2
    a = ((1 / 5) * r ** 2 * ang_vel ** 2 + (1 / 2) * lin_vel ** 2 + g * mu_k * delta * creepage) / denominator
    b = g * delta * sin(pitchAngle(position, delta)) / denominator
    print(b, a)
    return (b + sqrt(b ** 2 + 4 * a)) / 2
    
def nextLinVel(lin_vel, ang_vel, position, delta):
    # Equations modified to remove dependence on delta h
    r_roll = rollingRadius(position)
    mu_k = frictionCoefficient(position, delta_t)
    creepage = lin_vel - r_roll * ang_vel
    print(creepage)
    if creepage > 0 and ang_vel + (g * mu_k * delta) / r_roll < lin_vel / r_roll:
        a = lin_vel ** 2 - (4 / 5) * r ** 2 * ang_vel * (g * mu_k * delta / r_roll) - (2 / 5) * r ** 2 * (g * mu_k * delta / r_roll) ** 2 + 2 * g * mu_k * delta * creepage
        b = 2 * delta * g * sin(pitchAngle(position, delta))
        if (b ** 2 + 4 * a >= 0):
            print("hi")
            return (b + sqrt(b ** 2 + 4 * a)) / 2
        else:
            print("bye")
            return noSlip(lin_vel, ang_vel, position, delta, r_roll, mu_k, creepage)
    elif creepage < 0 and ang_vel - (g * mu_k * delta) / r_roll > lin_vel / r_roll:
        a = lin_vel ** 2 + (4 / 5) * r ** 2 * ang_vel * (g * mu_k * delta / r_roll) - (2 / 5) * r ** 2 * (g * mu_k * delta / r_roll) ** 2 + 2 * g * mu_k * delta * creepage
        b = 2 * delta * g * sin(pitchAngle(position, delta))
        if (b ** 2 + 4 * a >= 0):
            print("hello")
            return (b + sqrt(b ** 2 + 4 * a)) / 2
        else:
            print("goodbye")
            return noSlip(lin_vel, ang_vel, position, delta, r_roll, mu_k, creepage)
    else:
        return noSlip(lin_vel, ang_vel, position, delta, r_roll, mu_k, creepage)

def nextAngVel(next_lin_vel, ang_vel, position, delta):
    r_roll = rollingRadius(position)
    creepage = next_lin_vel - r_roll * ang_vel
    if creepage > 0:
        return min(next_lin_vel / r_roll, ang_vel + g * frictionCoefficient(position, delta_t) * delta / r_roll)
    else:
        return max(next_lin_vel / r_roll, ang_vel - g * frictionCoefficient(position, delta_t) * delta / r_roll)

import matplotlib.pyplot as plt
import numpy as np

delta_t = 0.01
array_len = int(10 / delta_t)
positions = np.zeros(array_len)
ang_vels = np.zeros(array_len)
lin_vels = np.zeros(array_len)
roll_radius_ls = np.zeros(array_len)
frictions = np.zeros(array_len)
position = 0
ang_vel = 0
lin_vel = 1
for i in range(0, array_len):
    #print(pitchAngle(position, delta_t))
    positions[i] = position
    ang_vels[i] = ang_vel
    lin_vels[i] = lin_vel
    if trackWidth(position + delta_t) >= r:
        break
    frictions[i] = frictionCoefficient(position, delta_t)
    roll_radius_ls[i] = rollingRadius(position)
    lin_vel = nextLinVel(lin_vel, ang_vel, position, delta_t)
    ang_vel = nextAngVel(lin_vel, ang_vel, position, delta_t)
    position += lin_vel * delta_t

print(positions)
plt.plot(ang_vels, label = "angular velocity")
plt.plot(lin_vels, label = "linear velocity")
plt.plot(lin_vels - ang_vels * roll_radius_ls, label = "creepage")
#plt.plot(frictions * g / roll_radius_ls)
#plt.plot(np.diff(ang_vels) / delta_t)
plt.legend()
plt.show()
