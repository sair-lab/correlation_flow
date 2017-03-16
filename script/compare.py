#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
from math import factorial

data1 = np.loadtxt("vicon_yaw.txt", skiprows=2)
time1 = data1[:,0] - data1[0,0]
vx_vicon = data1[:,1]
vy_vicon = -1*data1[:,2]
yaw = data1[:,3]
wz_vicon = np.zeros(len(yaw)-1)
for i in xrange (len(yaw)-1):
    wz_vicon[i] = (yaw[i+1]-yaw[i])/(time1[i+1]-time1[i])

data2 = np.loadtxt("px_vxvy3.txt", skiprows=2)
time2 = data2[:,0] - data1[0,0]
vx_of = data2[:,1]
vy_of = data2[:,2]

data3 = np.loadtxt("cf_yaw.txt", skiprows=0)
time3 = data3[:,0] - data1[0,0]
vx_cf = 0.75*data3[:,1]
vy_cf = 0.75*data3[:,2]
wz_cf = data3[:,3]

for i in xrange (4, len(vx_cf)):
    vx_cf[i] = 0.5*vx_cf[i]+0.2*vx_cf[i-1]+0.15*vx_cf[i-2]+0.10*vx_cf[i-3]+0.05*vx_cf[i-4]
    vy_cf[i] = 0.5*vy_cf[i]+0.2*vy_cf[i-1]+0.15*vy_cf[i-2]+0.10*vy_cf[i-3]+0.05*vy_cf[i-4]


#data curve smoothing filter
"""
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')
"""

if __name__ == "__main__":
#    vy_filt = savitzky_golay(vy_of, 51, 3)
    plt.figure()

    # plt.subplot(211)
    # plt.plot(time2, vx_of, 'y', label="PX4Flow")
    # plt.plot(time3, vx_cf, 'b', label="correlation flow")
    # plt.plot(time1, vx_vicon, c='r', linewidth=2.0, label="ground truth")
    # plt.xlabel("time[s]")
    # plt.ylabel("speed[m/s]")
    # legend = plt.legend(loc='upper right')

    # plt.subplot(212)
    # plt.plot(time2, vy_of, c='y', label="PX4Flow")
    # plt.plot(time3, vy_cf, c='b', label="correlation flow")
    # plt.plot(time1, vy_vicon, c='r', linewidth=2.0, label="ground truth")
    # plt.xlabel("time[s]")
    # plt.ylabel("speed[m/s]")
    # legend = plt.legend(loc='upper right')

    plt.plot(time3, wz_cf, 'y', label="correlation flow")
    plt.plot(time1[1:], wz_vicon, c='r', linewidth=2.0, label="ground truth")
    legend = plt.legend(loc='upper right')

    plt.show()
