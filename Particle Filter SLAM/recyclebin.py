# -*- coding: utf-8 -*-
"""
Created on Tue Feb 19 15:08:36 2019

@author: Mingchen Mao
"""

import os
current_dir = os.getcwd()
os.chdir(r'%s' %current_dir)

import numpy as np
import time
from matplotlib import pyplot as plt

from parameters import Parameters
para = Parameters()
import data_functions as df # data functions
import loca_functions as lf # localization functions
import plot_functions as pf # plot functions
from mapping import Mapping # mapping functions
ditu = Mapping(para)

#%% Dead Reckoning
# Preallocation
pp = np.zeros((1, 3))
pp40 = np.zeros((1, 3))
ppn = np.zeros((1, 3))

# Get pose
# no noise
ke = 0
for ki in range(0, len(omega)): # for each time
    if ki == 0: 
        p = np.reshape(np.array([0., 0., 0.]), (1, 3))
    if ti[ki] - te[ke] > 0 and ki != 0:
        ke += 1
    p = lf.predict__particles(p, v[ke], omega[ki], [0, 0], tao)
    pp = np.append(pp, p, axis = 0)
# sample at different rate
ke = 0
for ki in range(0, len(omega), 40): # for each time
    if ki == 0: 
        p = np.reshape(np.array([0., 0., 0.]), (1, 3))
    if ti[ki] - te[ke] > 0 and ki != 0:
        ke += 1
    p = lf.predict__particles(p, v[ke], omega[ki], [0, 0], tao)
    pp40 = np.append(pp40, p, axis = 0)
# with noise
ke = 0
for ki in range(0, len(omega)): # for each time
    if ki == 0: 
        p = np.reshape(np.array([0., 0., 0.]), (1, 3))
    if ti[ki] - te[ke] > 0 and ki != 0:
        ke += 1
    p = lf.predict__particles(p, v[ke], omega[ki], std, tao)
    ppn = np.append(ppn, p, axis = 0)
    
pp = np.delete(pp, 0, 0)
pp40 = np.delete(pp40, 0, 0)
ppn = np.delete(ppn, 0, 0)

# Plot
plt.figure(figsize = (20, 20))
plt.plot(pp[:, 0], pp[:, 1], '.')
plt.plot(pp40[:, 0], pp40[:, 1], '.')
plt.plot(ppn[:, 0], ppn[:, 1], '.')
plt.title('Dead Reckoning', fontsize = 20)
plt.legend(('No Noise', '1s Sample Rate', 'Yes Noise'), fontsize = 20)

#%% Old Get Transformation Matrix Function
#def get__pose(sp, sc, u):
#    """ get the pose matrix from 2 adjacent states
#    
#    Inputs:
#        sp = previous state [x, y, theta]
#        sc = current state [x, y, theta]
#    Outputs:
#        T = pose matrix transform something from body frame to world frame [[R, p], [0, 1]] """
#        
#    
#    # Compute pose matrix
#    px = sc[0] - np.cos(sc[2]) * sp[0] + np.sin(sc[2]) * sp[1] # x translation
#    py = sc[1] - np.sin(sc[2]) * sp[0] - np.cos(sc[2]) * sp[1] # y translation
#    p = np.reshape(np.stack((px, py)), (2, 1)) # translation matrix
#    R = np.vstack((np.hstack((np.cos(sc[2]), -np.sin(sc[2]))), 
#                   np.hstack((np.sin(sc[2]), np.cos(sc[2]))))) # rotation matrix
#    T = np.vstack((np.hstack((R, p)), [0, 0, 1]))
#    
#    return T

#%% Old Update States Function
#def update__state(s, u, tao):
#    """ plug measurements into the differential-drive model to update robot states
#    
#    Inputs:
#        s = previous state [x, y, theta]
#        u = current input = [averaged linear velocity, angular velocity]
#        tao = time step     
#    Outputs:
#        s = updated state [x, y, theta] """
#     
#        
#    # Compute each state    
#    s1 = tao * u[0] * np.sinc(u[1] * tao / 2) * np.cos(s[2] + u[1] * tao / 2)
#    s2 = tao * u[0] * np.sinc(u[1] * tao / 2) * np.sin(s[2] + u[1] * tao / 2)
#    s3 = tao * u[1]
#    s += np.array([s1, s2, s3])
#
#    
#    return s

#%% Old Map Correlation Code
#    # Convert from ranges to (x,y) coordinates
#    coor = [0, 0, 0]
#    x = [] # x, y are stored in case laser plots are called
#    y = []
#    
#    # Loop through each scan
#    for i in range(len(ran['r'][:, 0])): # is there a way to get rid of this for loop?
#        a = ran['a_min'] + ran['a_step'] * i # angle keeps increasing no matter scan valid or not
#        if ran['r'][i, k] > ran['r_min'] and ran['r'][i, k] < ran['r_max']: # get rid of too-close or too-far ranges
#            # transform to laser frame
#            x.append(ran['r'][i, k] * math.cos(a)) # every scan starts from (0, 0), -135 deg to 135 deg
#            y.append(ran['r'][i, k] * math.sin(a)) 
#            # transform to body frame
#            body = np.dot(para.T_bl, np.transpose([x[-1], y[-1], 1]))
#            # transform to world frame
#            R = np.vstack((np.hstack((np.cos(p[2]), -np.sin(p[2]))), 
#                           np.hstack((np.sin(p[2]), np.cos(p[2]))))) # rotation matrix
#            T_wb = np.vstack((np.hstack((R, [[p[0]], [p[1]]])), [0, 0, 1]))
#            coor = np.vstack((coor, np.dot(T_wb, body)))
#    coor = np.delete(coor, 0, axis = 0)

#%% Test range__to__xy
#z0=np.array([[0.,0.,0.],[0.,0.,-1/60*np.pi]])
#u0 = np.array([[10,0],[10, 0],[30,40]])
#ran = {"a_min": -1/3*np.pi, 
#       "a_step": 1/3*np.pi, 
#       "r_min": 0.01, 
#       "r_max": 30, 
#       "r": np.transpose(np.array([[10,5,20],[10,15,20]]))}
#
#k=1
#p=z0[k,:]
#
#display=True
## Convert range data to (x,y) coordinates
#a = ran['a_min'] + ran['a_step'] * np.arange(0, len(ran['r'][:, k]), 1) 
## transform to laser frame
#x3 = np.multiply(ran['r'][:, k], np.cos(a))
#x4 = np.multiply(ran['r'][:, k], np.sin(a))
#x5=tuple((ran['r'][:, k] > ran['r_min']) & (ran['r'][:, k] < ran['r_max']))
#x = x4[tuple((ran['r'][:, k] > ran['r_min']) & (ran['r'][:, k] < ran['r_max']))]
#y = x5[tuple((ran['r'][:, k] > ran['r_min']) & (ran['r'][:, k] < ran['r_max']))]
## transform to body frame
#s = np.vstack((x, y, np.ones((1, len(x[0, :])))))
#body = np.dot(para.T_bl, s)
## transform to world frame  
#R = np.vstack((np.hstack((np.cos(p[2]), -np.sin(p[2]))), 
#                   np.hstack((np.sin(p[2]), np.cos(p[2]))))) # rotation matrix
#T_wb = np.vstack((np.hstack((R, [[p[0]], [p[1]]])), [0, 0, 1]))
#coor = np.dot(T_wb, body)
#    
## Plot
#if display:
#    plt.figure()
#    plt.plot(body[0,:], body[1,:], 'b-d') # each scan 
#    plt.title('Laser Frame, Raw Scans')    
#    plt.plot(coor[0,:], coor[1,:], 'r-o') # each scan
#    ax=plt.gca()
#    ax.axis('equal')

#%% Old Process Data
## Read time data and define frequency
#""" Camera data will be dealt later """
#with np.load("Data/Encoders%d.npz"%para.dataset) as data:
#    te = data["time_stamps"] # encoder time stamps
#d = []
#for i in range(len(te) - 1):
#    d.append(te[i + 1] - te[i])
#fte = sum(d) / (len(te) - 1)
#fte = 1 / fte # sampling frequency (Hz)
#
#with np.load("Data/Hokuyo%d.npz"%para.dataset) as data:
#    tl = data["time_stamps"]  # acquisition times of the lidar scans
#d = []
#for i in range(len(tl) - 1):
#    d.append(tl[i + 1] - tl[i])
#ftl = sum(d) / (len(tl) - 1)
#ftl = 1 / ftl # sampling frequency (Hz)  
#    
#with np.load("Data/Imu%d.npz"%para.dataset) as data:
#    ti = data["time_stamps"]  # acquisition times of the imu measurements
#d = []
#for i in range(len(ti) - 1):
#    d.append(ti[i + 1] - ti[i])
#fti = sum(d) / (len(ti) - 1)
#fti = 1 / fti # sampling frequency (Hz)  
#  
#with np.load("Data/Kinect%d.npz"%para.dataset) as data:
#    td = data["disparity_time_stamps"] # acquisition times of the disparity images
#    tr = data["rgb_time_stamps"] # acquisition times of the rgb images   
#d = []
#for i in range(len(td) - 1):
#    d.append(td[i + 1] - td[i])
#ftd = sum(d) / (len(td) - 1)
#ftd = 1 / ftd # sampling frequency (Hz) 
#
#d = []
#for i in range(len(tr) - 1):
#    d.append(tr[i + 1] - tr[i])
#ftr = sum(d) / (len(tr) - 1)
#ftr = 1 / ftr # sampling frequency (Hz)  
# 
#print('f_Encoder = ' + str(round(fte)) + 'Hz\n' + 
#      'f_IMU = ' + str(round(fti)) + 'Hz\n' + 
#      'f_Lidar = ' + str(round(ftl)) + 'Hz\n')
#
#
## Discard insynchronized data
#name = ['Encoder', 'IMU', 'Lidar'] 
#temp = [te[0], ti[0], tl[0]]
#i0 = temp.index(max(temp)) # find the odometry that starts last
#temp = [te[-1], ti[-1], tl[-1]]
#i1 = temp.index(min(temp)) # find the odometry that ends first
#print(name[i0] + ' starts last')
#print(name[i1] + ' ends first')
#
#print('\nEncoder should start from its ' + str(abs(te - ti[0]).argmin()) + 'th element')
#print('Lidar should start from its ' + str(abs(tl - ti[0]).argmin()) + 'th element')
#
#"""
#print(12181) # IMU
#print((12181 - 1) / 5 * 2 + 1) # encoder
#print((12181 - 1) / 10 * 3 + 1) # camera
#"""
#    
#print('\nUse up to IMU[12181] gives "complete" data for all odometries\n' + 
#      'Encoder and Lidar has coorespondingly [4873], Camera is up to [3655]')
#
## Check
#te = te[12+372:4872+12-252] # 12 = start simultaneously with IMU, 372 = start moving, 252 = stop moving
#ti = ti[0+930:12180-630] # 930 = 372*2.5, 630 = 252*2.5
#tl = tl[12+372:4872+12-252]
#print('\nei start difference = ' + str(abs(te[0] - ti[0])) + 's\n' + 
#      'ei end difference = ' + str(abs(te[-1] - ti[-1])) + 's\n' + 
#      'li start difference = ' + str(abs(tl[0] - ti[0])) + 's\n' + 
#      'li end difference = ' + str(abs(tl[-1] - ti[-1])) + 's\n')

#%% Sample Omega At Encoder Rate
## Sample at lower frequency (encoder sampling rate)
#omega = np.zeros(int(len(w) / 2.5)) # IMU samples 2.5 times faster than encoder
#for i in range(len(omega)):
#    if i % 2 == 0: # even
#        omega[i] = w2[int(2.5 * i)]
#    else:
#        omega[i] = (w2[int(2.5 * i - 0.5)] + w2[int(2.5 * i + 0.5)]) / 2