# -*- coding: utf-8 -*-
"""
Created on Tue Feb 19 21:46:33 2019

@author: Mingchen Mao
"""


import os
current_dir = os.getcwd()
os.chdir(r'%s' %current_dir)

import numpy as np
import scipy.signal as ss
from matplotlib import pyplot as plt

from parameters import Parameters
para = Parameters()


## Manually call this to see how the encoder and IMU data differ
#plt.figure()
#plt.plot(np.subtract(v_r, v_l) / para.L * para.tao_enc)
#plt.plot(omega * para.tao_imu)
#tit = '2 lines should perfectly match, but the max difference is about 2.3 deg, should be OK'
#plt.title(tit.title(), fontsize = 20)
#plt.ylabel('Angle in Radian')
    

## Manually call this to load all data
#with np.load("Data/Encoders%d.npz"%para.dataset) as data:
#    encoder_counts = data["counts"] # 4 x n encoder counts
#    encoder_stamps = data["time_stamps"] # encoder time stamps
#
#with np.load("Data/Hokuyo%d.npz"%para.dataset) as data:
#    lidar_angle_min = data["angle_min"] # start angle of the scan [rad]
#    lidar_angle_max = data["angle_max"] # end angle of the scan [rad]
#    lidar_angle_increment = data["angle_increment"] # angular distance between measurements [rad]
#    lidar_range_min = data["range_min"] # minimum range value [m]
#    lidar_range_max = data["range_max"] # maximum range value [m]
#    lidar_ranges = data["ranges"]       # range data [m] (Note: values < range_min or > range_max should be discarded)
#    lidar_stamsp = data["time_stamps"]  # acquisition times of the lidar scans
#
#with np.load("Data/Imu%d.npz"%para.dataset) as data:
#    imu_angular_velocity = data["angular_velocity"] # angular velocity in rad/sec
#    imu_linear_acceleration = data["linear_acceleration"] # Accelerations in gs (gravity acceleration scaling)
#    imu_stamps = data["time_stamps"]  # acquisition times of the imu measurements
#  
#with np.load("Data/Kinect%d.npz"%para.dataset) as data:
#    disp_stamps = data["disparity_time_stamps"] # acquisition times of the disparity images
#    rgb_stamps = data["rgb_time_stamps"] # acquisition times of the rgb images
        


def initial__check():
    """ understand the raw data, make data synchronized """
    
    
    # Load data
    with np.load("Data/Encoders%d.npz"%para.dataset) as data:
        e = data["counts"] # 4 x n encoder counts
        te = data["time_stamps"] # encoder time stamps
    d = []
    for i in range(len(te) - 1):
        d.append(te[i + 1] - te[i])
    fte = sum(d) / (len(te) - 1)
    fte = 1 / fte # sampling frequency (Hz)
        
    with np.load("Data/Imu%d.npz"%para.dataset) as data:
        ti = data["time_stamps"]  # acquisition times of the imu measurements
    d = []
    for i in range(len(ti) - 1):
        d.append(ti[i + 1] - ti[i])
    fti = sum(d) / (len(ti) - 1)
    fti = 1 / fti # sampling frequency (Hz)  
    
    with np.load("Data/Hokuyo%d.npz"%para.dataset) as data:
        tl = data["time_stamps"]  # acquisition times of the lidar scans
    d = []
    for i in range(len(tl) - 1):
        d.append(tl[i + 1] - tl[i])
    ftl = sum(d) / (len(tl) - 1)
    ftl = 1 / ftl # sampling frequency (Hz)  
    
    print('f_Encoder = ' + str(round(fte)) + 'Hz\n' + 
          'f_IMU = ' + str(round(fti)) + 'Hz\n' + 
          'f_Lidar = ' + str(round(ftl)) + 'Hz\n')
    
    # Discard useless data (robot not moving) and insynchronized data
    e0 = sum(e) # sum of each wheel's count
    ie = np.argwhere(abs(e0) > 4) # averagely 1 count on each wheel ~= moving
    te = te[ie[0, 0] : ie[-1, 0]]
    # get truncate indices
    trun_e = [ie[0, 0], ie[-1, 0]]
    trun_i = [abs(te[0] - ti).argmin(), abs(te[-1] - ti).argmin()]
    trun_l = [abs(te[0] - tl).argmin(), abs(te[-1] - tl).argmin()]
    trun = [trun_e, trun_i, trun_l] 
    # check 
    ti = ti[abs(te[0] - ti).argmin() : abs(te[-1] - ti).argmin()]
    tl = tl[abs(te[0] - tl).argmin() : abs(te[-1] - tl).argmin()] # device measure time matches
    name = ['Encoder', 'IMU', 'Lidar'] 
    temp = [te[0], ti[0], tl[0]]
    i0 = temp.index(max(temp)) # find the odometry that starts last
    temp = [te[-1], ti[-1], tl[-1]]
    i1 = temp.index(min(temp)) # find the odometry that ends first
    print(name[i0] + ' starts last')
    print(name[i1] + ' ends first')
    print('\ne_start - i_start = ' + str(te[0] - ti[0]) + 's\n' + 
          'e_end - i_end = ' + str(te[-1] - ti[-1]) + 's\n' + 
          'e_start - l_start = ' + str(te[0] - tl[0]) + 's\n' + 
          'e_end - l_end = ' + str(te[-1] - tl[-1]) + 's\n' + 
          'l_start - i_start = ' + str(tl[0] - ti[0]) + 's\n' + 
          'l_end - i_end = ' + str(tl[-1] - ti[-1]) + 's\n')

    print('To see the complete raw data, uncomment and run codes inside "data_functions.py" \n')

    return trun



def do__encoder(para, trun):
    """ calculate average linear velocities from encoder measurements
    
    Inputs:
        para = associated parameters in the Parameters class
        trun = truncate indices
    Outputs:
        v = average linear velocities of the differential drive measured by the encoder """
    
    
    # Load data
    with np.load("Data/Encoders%d.npz"%para.dataset) as data:
        e = data["counts"] # dimension 4 = [FR FL RR RL]
        te = data["time_stamps"] # encoder time stamps
    e = e[:, trun[0][0] : trun[0][1]] # counts
    te = te[trun[0][0] : trun[0][1]]
    
    # Calculate linear velocities
    v_l = [] # left wheel velocity
    v_r = [] # right wheel velocity   
    for i in range(len(e[0, :])):
        v_r.append((e[0, i] + e[2, i]) / 2 * 0.0022 / para.tao_enc) # average out measurement error
        v_l.append((e[1, i] + e[3, i]) / 2 * 0.0022 / para.tao_enc)
    
    # Calculate average linear velocity
    v = np.add(v_l, v_r) / 2 
    
    return v, te



def do__IMU(para, trun, display = False):
    """ apply a low-pass filter for noise reduction, sample IMU at encoder/lidar rate
    
    Inputs:
        para = associated parameters in the Parameters class 
        trun = truncate indices
        display = whether or not to display a plot showing effects of low-pass filter and low-rate sampling
    Outputs:
        omega = angular velocities measured by IMU sampled at the same rate as encoder """
        
    
    # Load data
    with np.load("Data/Imu%d.npz"%para.dataset) as data:
        w = data["angular_velocity"] # angular velocity (rad/s)
        ti = data["time_stamps"]  # acquisition times of the imu measurements
    w = w[2, trun[1][0] : trun[1][1]] # yaw rate (rad/s)
    ti = ti[trun[1][0] : trun[1][1]]
    
    # Apply low-pass filter
    num, den = ss.butter(1, para.lowpass / 50, btype = 'low') # 1 = 100 Hz / 2 = Nyquist
    omega = ss.filtfilt(num, den, w)
  
    # Plot
    if display:
        plt.figure()
        plt.plot(np.arange(0, len(w), 1), w)
        plt.plot(np.arange(0, len(w), len(w) / len(omega)), omega)
        plt.title('Effects of Low-Pass Filter & Low-Sampling Rate')
           
    return omega, ti
    

    
def do__lidar(trun):
    """ save lidar data to a dictionary
    
    Inputs:
        trun = truncate indices
    Outputs:
        ran = a dictionary with lidar data [a_min, a_step, r_min, r_max, ranges] """


    # Load data
    with np.load("Data/Hokuyo%d.npz"%para.dataset) as data:
        lidar_angle_min = data["angle_min"] # start angle of the scan (rad)
        lidar_angle_increment = data["angle_increment"] # angular distance between measurements (rad)
        lidar_range_min = data["range_min"] # minimum range value (m)
        lidar_range_max = data["range_max"] # maximum range value (m) 
        lidar_ranges = data["ranges"] # range data (m)
        tl = data["time_stamps"]  # acquisition times of the lidar scans
    lidar_ranges = lidar_ranges[:, trun[2][0] : trun[2][1]] # [# of measurements in each scan] * [# of scans in total time] 
    tl = tl[trun[2][0] : trun[2][1]]   
        
    # Build dictionary
    ran = {"a_min": np.float(lidar_angle_min), # convert this to a number to avoid future syntax error
           "a_step": np.float(lidar_angle_increment), 
           "r_min": np.float(lidar_range_min), 
           "r_max": np.float(lidar_range_max), 
           "r": lidar_ranges}
    
    return ran, tl
    










