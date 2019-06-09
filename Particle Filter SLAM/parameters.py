# -*- coding: utf-8 -*-
"""
Created on Mon Feb 18 19:08:00 2019

@author: Mingchen Mao
"""

import numpy as np



class Parameters():
    """ Store parameters that may be useful in this particle filter SLAM """
    
    
    def __init__(self):
        
        # Logistics
        self.dataset = 23 # which dataset to use 
        self.tao_enc = 1 / 40 # encoder time step
        self.tao_imu = 1 / 100
        self.lowpass = 5 # cutoff frequency of the low-pass filter (Hz)
        
        # Map
        self.cell_size = 0.3 # (m)
        self.cell_num = 301 # there are totally 'cell_num * cell_num' cells, odd number gives (0,0) at center
        self.trust_1 = 0.77 / 0.23 # p(we trust laser hits the wall) / p(we trust laser behaves)
        self.trust_0 = 1 / self.trust_1 
        # log-odds limits to avoid over-confident estimation
        prob_min = 0.01 # a cell has at least this probability to be occupied
        prob_max = 0.99 # a cell has at most this chance to be occupied
        self.logodds_min = np.log(prob_min / (1 - prob_min))
        self.logodds_max = np.log(prob_max / (1 - prob_max))
        # map correlation grid size
        self.var = 3 # check self.var*self.var cells around one point
        
        # Pose
        self.T_bl = [[1, 0, 0.29833], [0, 1, 0.51435], [0, 0, 1]] # transform matrix from Laser frame to Body frame
        self.L = 0.3937 # distance between 2 parallel wheels (m)
        
        # Particle
        self.part_num = 100
        
        
        
        