# -*- coding: utf-8 -*-
"""
Created on Mon Feb 18 10:19:33 2019

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


def tic():
    """ tic """
    
    return time.time()

def toc(t_start, name = "Operation"):
    """ toc
    
    Inputs:
        t_start = the time returned by tic()
        name = task name you want to display """
        
    print('%s took: %.8f sec.\n' % (name, (time.time() - t_start)))
    
#%% Process Data

trun = df.initial__check() # get indices to truncate
v, te = df.do__encoder(para, trun) # overall linear velocity
omega, ti = df.do__IMU(para, trun, display = False) # yaw rate
ranges, tl = df.do__lidar(trun) # "length" of laser beams
std = [np.std(v), np.std(omega) / 20] # standard deviation of inputs
tao = 1 / 100 # time step (s)

#%% SLAM  
# Preallocations
map_all = [] # store maps at each time step
part_b = np.zeros((1, 3)) # store best particle poses at each time step
ke = 0 # encoder time step
kl = 0 # Lidar time step

# Loop through time
ts = tic()
for ki in range(0, len(omega)): # for each Lidar sample
    # Prior
    if ki == 0: 
        part_all, wei = lf.create__particles((-1, 1), (-1, 1), (0, 0), para.part_num)
        part = [0., 0., 0.] # the starting coordinates was selected to be (-12, -12)
        T = np.vstack((np.hstack((np.identity(2), [[0], [0]])), [0, 0, 1]))
    
    # Change indices
    if ti[ki] - tl[kl] > 0 and ki != 0: # use new measurement, ki != 0 prevent te[0] < ti[0]
        kl += 1
    if ti[ki] - te[ke] > 0 and ki != 0:
        ke += 1
    if kl == len(tl) or ke == len(te): # one of these data run out
        break  
        
    # Update log-odds map
    # convert laser measurements to world frame using the "best" particle from last time step
    xy_world = lf.range__to__xy(para, ranges, part, kl, display = False) # put inside loop b.c. # of valid beams varies with time
    ori = [part[0] / para.cell_size, part[1] / para.cell_size] # origin (coordinate of Lidar)
    # compute occupancy-grid map
    for j in range(len(xy_world[:, 0])): # for each beam
        xy2 = xy_world[j, :] / para.cell_size # scale xy up because Bresenham only takes integer 
        xy_Bres = ditu.Bresenham2D([ori[0], ori[1]], [xy2[0], xy2[1]]) 
        log_odds = ditu.update__cells(xy_Bres) # prior of log-odds map was done in class __init__ 
#    # eliminate extreme log-odds
#    log_odds[np.where(log_odds < para.logodds_min)] = para.logodds_min 
#    log_odds[np.where(log_odds > para.logodds_max)] = para.logodds_max
    # store maps for plot
    map_all.append(1 - 1 / (1 + np.exp(log_odds))) # convert to pdf of being occupied for each cell

    # Update particles
    # apply map correlation model
    corre = np.zeros(len(part_all[:, 0])) 
    for i in range(len(part_all[:, 0])): # for each particle
        xy_world = lf.range__to__xy(para, ranges, part_all[i, :], kl)
        corre[i] = lf.get__correlation(para, log_odds.copy(), xy_world) # coorelation score of each particle
    wei = lf.update__weights(corre, wei) # weight
    # resample if necessary
    resampled = False
    if 1 / sum(wei**2) < para.part_num / 3: # use N/3 as threshold
        part_all, wei = lf.stratified__resampling(part_all, wei) # use stratified resampling
        resampled = True
    
    # Predict particles
    # move particles using current input
    part_all = lf.predict__particles(part_all, v[ke], omega[ki], std, tao)
    # return the pose of the best particle
    if resampled:
        part = part_all[-1, :] # stratified resampling puts small-weight particles at the beginning of array
    else:
        part = part_all[np.argmax(wei), :]
    # store particle poses for plot
    part_b = np.append(part_b, np.reshape(part, (1, len(part))), axis = 0)

    toc(ts,'c' + str(ki))

np.save('mapall', map_all)
np.save('partb', part_b)

#%% Animated Plot
## Load results
map_all = np.load('mapall.npy')
part_b = np.load('partb.npy')

# Axes work
# show correct ticks (x: No. of cells count from the left, y: No. of cells count from top) and labels
fig, ax = plt.subplots(figsize = (20, 20))
tick_i = (para.cell_num - 1) / 10 # tick increment, 10 is roughly how many ticks to show on axes
tick_li = (para.cell_num - 1) * para.cell_size / 10 # label increment
tick_s = (para.cell_num - 1) / 2 * para.cell_size # side ticks (map wall)
# first arg is for the real ticks, second arg is for masks
plt.xticks(np.arange(0, para.cell_num, tick_i), 
           np.arange(-tick_s, tick_s + tick_li, tick_li), fontsize = 20) # make tick labels meet physical truth
plt.yticks(np.arange(0, para.cell_num, tick_i), np.arange(tick_s, -(tick_s + tick_li), -tick_li), fontsize = 20)
# get axes ranges
xl = ax.get_xlim()
yl = ax.get_ylim()

# Animated plot
# get objects needed to be animated
image = plt.imshow(map_all[0] , cmap = 'Greys')
robot, = ax.plot([], [], 'ro', markersize = 3)
timecount = ax.text(0.1 * (xl[1] - xl[0]), 0.1 * (yl[1] - yl[0]), '', fontsize = 30, color = 'b')
legendrobo, = ax.plot(0.07 * (xl[1] - xl[0]), 0.9 * (yl[1] - yl[0]), 'ro', markersize = 10) # legend is faked
legendtextr = ax.text(0.1 * (xl[1] - xl[0]), 0.91 * (yl[1] - yl[0]), '', fontsize = 30, color = 'g')
legendwall, = ax.plot(0.07 * (xl[1] - xl[0]), 0.93 * (yl[1] - yl[0]), 'ks', markersize = 10) # legend is faked
legendtextw = ax.text(0.1 * (xl[1] - xl[0]), 0.94 * (yl[1] - yl[0]), '', fontsize = 30, color = 'g')
# get data used to update above objects
x = np.int16(np.round((part_b[:, 0] + para.cell_size * (para.cell_num - 1) / 2) / para.cell_size)) # convert from position to index
y = np.int16(np.round((para.cell_size * (para.cell_num - 1) / 2 - part_b[:, 1]) / para.cell_size))
time_elap = np.arange(0, len(omega) * 0.01, 0.01) # starts from 0 s, incremented by 0.025 s
# decorate plot
plt.xlabel('(m)', fontsize = 25)
plt.ylabel('(m)', fontsize = 25)
plt.title('Particle Filter SLAM', fontsize = 35)
# plot and save
gif = pf.plot__animatedly(para, fig, ax, image, robot, timecount, legendrobo, legendtextr, legendwall, 
                          legendtextw, map_all, np.stack((x, y)).T, time_elap)    
gif.save('turn off the radio.mp4', fps = 30, extra_args = ['-vcodec', 'libx264'])  
#plt.close('all')
#from IPython import get_ipython  
#get_ipython().magic('reset -f')

