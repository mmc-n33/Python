# -*- coding: utf-8 -*-
"""
Created on Mon Jan 28 09:42:19 2019

@author: Mingchen Mao
"""

import os
current_dir = os.getcwd()
os.chdir(r'%s' %current_dir)

import numpy as np
from matplotlib import image as img
from matplotlib import pyplot as plt

from barreldetector import BarrelDetector
from parameters import Parameters
parameter = Parameters() 
import trainingfunctions as tf


#%% Obtain training set
# Uncomment this cell to obtain a new model

"""
# Get data of barrel-blue and colors similar to barrel-blue
tf.obtain_training_set_color('give_a_name')
"""

#%% Obtain training model
algo = 'LR' # define the algorithm 
# Train parameters of barrel-contour shape, then manually change these parameters in parameters.py
maxr, minr = tf.obtain_training_set_shape(parameter, algo)
print('max_ratio = ' + str(maxr), 'min_ratio = ' + str(minr))

# Train parameters for color model, then automatically saved as a .npy file
bd = BarrelDetector(parameter, 1)
bd.segment_image('aaa', algo, display = False)


#%% Test
d = os.listdir('Images/testset')
box_list = []

for i in range(len(d)):
    barrel_detector = BarrelDetector(parameter, img.imread('Images/testset/' + d[i]))
    tub = barrel_detector.segment_image(alg = 'GNB', display = True) # get binary image
    bounding_box, z = barrel_detector.get_bounding_box(tub) # get bounding box
    if z == 1: # if test failed
        print('No blue barrel detected in ' + d[i] + '!')
    box_list.append(bounding_box) # store all boxes info


#%% Close test images
#plt.close("all")
