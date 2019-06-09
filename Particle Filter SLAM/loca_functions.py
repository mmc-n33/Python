# -*- coding: utf-8 -*-
"""
Created on Wed Feb 20 22:43:55 2019

@author: Mingchen Mao
"""

import numpy as np
from matplotlib import pyplot as plt


    
def range__to__xy(para, ran, p, k, display = False):
    """ Convert ranges to (x, y) coordinates in the world frame
    
    Inputs:
        para = associated parameters in the Parameters class
        ran = range-related data at current time [a_min, a_step, r_min, r_max, ranges]
        p = current pose [x, y, theta]
        k = current time step index
        display = False means do not display laser scan plot 
    Outputs:
        coor = (x, y) coordinates of laser beams' tips in world frame, [x, y, 1] """
    
    
    # Convert range data to (x,y) coordinates
    # scan angles
    a = ran['a_min'] + ran['a_step'] * np.arange(0, len(ran['r'][:, k]), 1) 
    # transform to laser frame
    x_raw = np.multiply(ran['r'][:, k], np.cos(a))
    y_raw = np.multiply(ran['r'][:, k], np.sin(a))
    x = x_raw[(ran['r'][:, k] > ran['r_min']) & (ran['r'][:, k] < ran['r_max'])] # eliminate too-close/far beams
    y = y_raw[(ran['r'][:, k] > ran['r_min']) & (ran['r'][:, k] < ran['r_max'])]
    x = np.reshape(x, (1, len(x))) # for future matrix multiplification
    y = np.reshape(y, (1, len(y)))
    # transform to body frame
    s = np.vstack((x, y, np.ones((1, len(x[0, :]))))) 
    body = np.dot(para.T_bl, s)
    # transform to world frame  
    R = np.vstack((np.hstack((np.cos(p[2]), -np.sin(p[2]))), 
                       np.hstack((np.sin(p[2]), np.cos(p[2]))))) # rotation matrix
    T_wb = np.vstack((np.hstack((R, [[p[0]], [p[1]]])), [0, 0, 1]))
    coor = np.dot(T_wb, body)
   
    # Plot beams
    if display:
        plt.figure()
        for i in range(len(coor[0, :])): # for each bream
            plt.plot([0, x_raw[i]], [0, y_raw[i]], 'b') 
        plt.plot(0, 0, 'ko', markerfacecolor = 'r', markersize = 20) # Lidar
        plt.title('Laser Frame, Raw Scans')

    return coor.T



def create__particles(x_r, y_r, theta_r, N):
    """ generate randomly distributed particles and corresponding weights
    
    Inputs:
        x_r = possible ranges [lower and upper] of x coordinates (m)
        y_r = possible ranges [lower and upper] of y coordinates (m)
        theta_r = possible ranges [lower and upper] of yaw angle (rad)
        N = # of particles 
    Outputs:
        p = randomly generated particles [# of particles, [[x], [y], [theta]]]
        weights = 1 / # of particles """
        
    
    # Randomly generate x, y and theta under uniform distributions 
    p = np.empty((N, 3))
    p[:, 0] = np.random.uniform(x_r[0], x_r[1], N)
    p[:, 1] = np.random.uniform(y_r[0], y_r[1], N)
    p[:, 2] = np.random.uniform(theta_r[0], theta_r[1], N)

    return p, 1 / N * np.ones(N)



def predict__particles(p, v, omega, std, tao):
    """ predict particle states based on inputs to robot """
     

    # Create a list of identical u so that each particle gets different noise
    u = np.stack((v, omega))
    uu0 = u[0] * np.ones(len(p))
    uu1 = u[1] * np.ones(len(p))
    uu = np.stack((uu0, uu1)) # it is a list of u, like uuuuuuuuuuu, so named "uu"
    
    # Add noise to inputs
    uu[0, :] += std[0] * np.random.randn(len(p)) # assuming zero noise mean
    uu[1, :] += std[1] * np.random.randn(len(p))
    
    # Apply the differential drive model to predict particles' poses
    p[:, 0] += tao * uu[0, :] * np.sinc(uu[1, :] * tao / 2) * np.cos(p[:, 2] + uu[1, :] * tao / 2)    
    p[:, 1] += tao * uu[0, :] * np.sinc(uu[1, :] * tao / 2) * np.sin(p[:, 2] + uu[1, :] * tao / 2)  
    p[:, 2] += tao * uu[1, :]
    

    return p



def get__correlation(para, m, coor):
    """ compute correlation between each particle and the previously best map
    
    Inputs:
        para = associated parameters in the Parameters class
        m = log-odds map based on previous best particle
        coor = occupied cells' coordinates in world frame based on current particle
    Outputs:
        corre = max correlation between old map and particle new map """
       
        
    # Convert log-odds map into a form that is easy to count                     
    m[m > np.log(0.5)] = 0.2 # occupied cells' weight
    m[m <= np.log(0.5)] = 0 # free cells' weight
    
    # Define grid window size (make sure get good correlation) and total map size
    xy_check = np.arange(-(para.var - 1) / 2 * para.cell_size, 
                        (para.var + 1) / 2 * para.cell_size, para.cell_size)
    xy_max = para.cell_size * (para.cell_num - 1) / 2

    # Compute correlation
    corre = 0
    for i in range(para.var): # for each cell in the grid window
        ix = np.int16(np.round((coor[:, 0] + xy_check[i] + 
                                xy_max) / para.cell_size)) # # of cells between current particle and the left wall
        for j in range(para.var):
            iy = np.int16(np.round((xy_max - coor[:, 1] - 
                                    xy_check[j]) / para.cell_size)) # # of cells between current particle and the top wall
            # make sure 0 < indices < indices of map wall
            xgood = np.logical_and((ix >= 0), (ix < para.cell_num))
            ygood = np.logical_and((iy >= 0), (iy < para.cell_num))
            good = np.logical_and(xgood, ygood)
            # +1 if find an occupied cell matching current laser measurments at current particle
            if np.sum(m[iy[good], ix[good]]) > corre: # ix prepresents y index
                corre = np.sum(m[iy[good], ix[good]])

    return corre



def update__weights(corre, w_old):
    """ calculate updated weights for each particle """
        
        
    # Use softmax to compute weights (probability model of each particle)
    ph = np.empty(len(corre))
    for i in range(len(corre)):
        ph[i] = 1 / sum(np.exp(corre - corre[i])) # divide by the numerator to avoid exp(large) = inf
        
    # Normalize weights
    alpha = w_old * ph / sum(w_old * ph)
    
    
    return alpha
            
            

def stratified__resampling(p, w):
    """ perform a stratified resampling if particles are bad """
        
    
    # Apply the stratified resampling
    N = len(w)
    pr = p
    wr = 1 / N * np.ones(N)
    for i in range(N):     
        j = 0
        cum = w[j] # cumulative weight
        a = np.random.uniform(0, 1 / N, 1)
        b = a + (i - 1) / N
        while b > cum:
            j += 1
            cum += w[j]
        pr[i, :] = p[j, :]
        
    
    return pr, wr
    
                  
            
          
            
            
            
            
            
            
            