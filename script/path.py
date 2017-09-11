#!/usr/bin/env python

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import math
from math import factorial
import tf


data2 = np.loadtxt("/home/jitete/drones/src/correlation_flow/bag/results/rect_viconpose.txt", skiprows=2)
time2 = data2[:,0] - data2[0,0]
px_vicon = data2[:,1] - data2[0,1]
py_vicon = data2[:,2] - data2[0,2]
pz_vicon = data2[:,3]

data3 = np.loadtxt("/home/jitete/drones/src/correlation_flow/bag/results/rect_cf.txt", skiprows=2)
time3 = data3[:,0] - data3[0,0]
vx_cf = data3[:,1]
vy_cf = data3[:,2]
vz_cf = data3[:,3]


px_cf = np.zeros(len(vx_cf))
py_cf = np.zeros(len(vy_cf))

for i in xrange (len(px_cf)-1):
    px_cf[i+1] = px_cf[i] + vx_cf[i]*(time3[i+1]-time3[i])
    py_cf[i+1] = py_cf[i] + vy_cf[i]*(time3[i+1]-time3[i])


outtxt = open('/home/jitete/drones/src/correlation_flow/bag/results/rect_cfpose.txt','w')
outtxt.write('# text file' + '\n# format: time stamp x y z qx qy qz qw\n')
for i in xrange (len(px_cf)):
    outtxt.write(str.format("{0:.9f} ", data3[i,0]))
    outtxt.write(str.format("{0:.9f} ", px_cf[i]))
    outtxt.write(str.format("{0:.9f} ", py_cf[i]))
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0\n')
outtxt.close()

if __name__ == "__main__":
    fig = plt.figure()

    plt.plot(px_cf, py_cf, 'b', label="correlation flow")
    plt.plot(px_vicon, py_vicon, 'r', label="vicon")

    plt.xlabel("position-x", fontsize=18)
    plt.ylabel("position-y", fontsize=18)
    legend = plt.legend(loc='lower right')

    plt.xticks(np.arange(-0.5, 3.5, 0.5), fontsize=14)
    plt.yticks(fontsize=14)

    plt.grid()
    plt.show()
