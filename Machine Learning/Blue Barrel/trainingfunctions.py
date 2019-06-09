# -*- coding: utf-8 -*-
"""
Created on Sat Jan 26 17:28:20 2019

@author: Mingchen Mao
"""

import os
current_dir = os.getcwd()
os.chdir(r'%s'%current_dir)

import numpy as np
#import keyboard as kb
from matplotlib import pyplot as plt
from matplotlib import image as img

from roipoly import RoiPoly

import cv2
from skimage.measure import label, regionprops


def obtain_training_set_color(savename):
    """ hand-label positive and negative regions based on their colors, save corresponding coordinates """
    
    
    print('The function "obtain_training_set_color" does not work in Spyder \n')
    print('The function "obtain_training_set_color" does not work in Spyder \n')
    print('The function "obtain_training_set_color" does not work in Spyder \n')
    # Preliminaries
    z = os.listdir('Images/trainset') # image directory
    x = {} # create a dictionary
    cl = 0 # decide whether to obtain a new training set
    
    # User interaction
    while True:
        print('You are about to create a new training set, please put training images under the folder "trainset" \n')
        print('Did you copy images into the "trainset" folder? Y/N \n')
        y = input()
        if y == 'y' or y == 'Y':
            break
        elif y == 'n' or y == 'N':
            cl = 1 # do not run what is next
            break
        else:
            print('\n\n wtf??? \n\n')
            continue
    
    # Load image, draw roi, get coordinates, save data
    if cl == 0:
        print('red roi = true, yellow roi = false') # user instruction
        
        for i in range(len(z)):
            tu = img.imread('Images/trainset/' + z[i])
            train_1 = [0, 0 ,1]
            train_0 = [0, 1, 0]
            
            while True:
                plt.figure()
                fig = plt.get_current_fig_manager()
                fig.window.setGeometry(400, 100, 3000, 2000)
                plt.imshow(tu)
                my_roi1 = RoiPoly(color = 'r') # call this to draw
        
                # create training set
                mask1 = my_roi1.get_mask(tu[:,:,0]) * 1 # what's inside roi is 1, outside is 0
                ind = np.where(mask1 == 1) # get 1(s) indices
                train_1 = np.vstack((train_1, tu[ind])) # get space coordinates corresponding to blue(1)
                
                kb = input('if you have done labeling this image, press "q" \n Or press any other buttons to continue labeling this image')
                if kb == 'q': # did not use direct keyboard detection because it conflicts with RoiPoly
                    break
            
            # save training set for each image with different keys, maybe useful for future recovery and analysis?
            key = 'shi' + str(i)
            x[key] = train_1
            
            while True:
                plt.figure()
                fig = plt.get_current_fig_manager()
                fig.window.setGeometry(400, 100, 3000, 2000)
                plt.imshow(tu)
                my_roi0 = RoiPoly(color = 'y') # call this to draw
                
                mask0 = my_roi0.get_mask(tu[:,:,0]) * 1 # what's inside roi is 1, outside is 0
                ind = np.where(mask0 == 1) # get 1(s) indices
                train_0 = np.vstack((train_0, tu[ind])) # get space coordinates corresponding to non-blue(0)
                
                kb = input('if you have done labeling this image, press "q" \n Or press any other buttons to continue labeling this image')
                if kb == 'q': # did not use direct keyboard detection because it conflicts with RoiPoly
                    break
            
            # save training set for each image with different keys, maybe useful for future recovery and analysis?
            key = 'bushi' + str(i)
            x[key] = train_0

            np.save(savename, x)



def obtain_training_parameters(para, x, y, alg = 'LR'):
    """ given training set, return training parameters """
    
    
    global omega
    
    # Iterate to find the optimal parameters
    if alg == 'LR': # logistic regression
        omega = np.zeros((3, 1))
        alpha = para.step_size # step size
        for i in range(para.iteration):
                grad = np.zeros((3, 1))
                for i in range(len(x[:, 0])):
                    grad += np.reshape(x[i, :], (3, 1)) * (-y[i] + 1 / (1 + np.exp(-np.dot(x[i, :], omega))))
                omega -= alpha * grad 
                
    elif alg == 'GNB': # Gaussian Naive Bayes
        # get counts for each class
        itszero = 0
        itsone = 0
        for i in range(len(y)):
            if y[i] == 1:
                itsone += 1
            else:
                itszero += 1
                
        # probability of see y
        theta0 = itszero / len(y)
        theta1 = 1 - theta0
        
        # mean of omega
        mew00 = 0
        mew01 = 0
        mew02 = 0
        mew10 = 0
        mew11 = 0
        mew12 = 0
        for i in range(len(y)):
            if y[i] == 0:
                mew00 += x[i, 0] / itszero
                mew01 += x[i, 1] / itszero
                mew02 += x[i, 2] / itszero
            else:
                mew10 += x[i, 0] / itsone
                mew11 += x[i, 1] / itsone
                mew12 += x[i, 2] / itsone
        
        # variance of omega    
        sigma00 = 0
        sigma01 = 0
        sigma02 = 0
        sigma10 = 0
        sigma11 = 0
        sigma12 = 0
        for i in range(len(y)):
            if y[i] == 0:
                sigma00 += (x[i, 0] - mew00)**2 / itszero
                sigma01 += (x[i, 1] - mew01)**2 / itszero
                sigma02 += (x[i, 2] - mew02)**2 / itszero
            else:
                sigma10 += (x[i, 0] - mew10)**2 / itsone
                sigma11 += (x[i, 1] - mew11)**2 / itsone
                sigma12 += (x[i, 2] - mew12)**2 / itsone
        
        # store these parameters into the name "omage"
        omega = [theta0, theta1, mew00, mew01, mew02, mew10, mew11, mew12,
                 sigma00, sigma01, sigma02, sigma10, sigma11, sigma12] 
        
    else: # Gaussian Mixture
        pass
    
    return omega
 

               
def obtain_training_set_shape(para, alg):
    """ use image examples to determine the shape of bounding box """
    
    
    # Preliminaries
    z = os.listdir('Images/shapeset') # image directory
    box_how = [] # the ratio of box's height over its width
    omega = np.load('omega' + alg + '.npy') # load parameters
    
    # Establish a typical bounding box shape
    for i in range(len(z)):
        tu = img.imread('Images/shapeset/' + z[i])
        tu_b = obtain_testing_y(tu, omega, alg)
        tu_b = tu_b.astype(np.uint8) # convert binary image to a format that @findContours can process
    
        # find contours of objects with wanted color
        contours, hierachy = cv2.findContours(tu_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # the binary image will be replaced by this binary contour image
        cv2.drawContours(tu_b, contours, -1, (255, 0, 0), 3) # -1 = draw all contours, (color), thickness of contour lines
      
        # get contours edges, namely bounding box
        tu_b = label(tu_b) # label connected regions of an integer array, so that unconnected contours will considered as separate regions
        region = regionprops(tu_b) # identify regions of the labeled image
        rc = [] # region's centroids

        # get rid of tiny regions
        for prop in region.copy():
            if prop.bbox_area < para.bbox_area:
                region.remove(prop)
            else:
                rc.append(prop.centroid)
        
        # get rid of repeated regions
        ind = sorted(range(len(rc)), key = rc.__getitem__) # store element indices of local_centroid tuples before sorting
        rs = sorted(rc) # sorted region

        rdel = [] # repeated regions to be deleted
        for i in range(0, len(rs) - 1):
            if abs(rs[i+1][0] - rs[i][0]) < para.cent_dif and abs(rs[i+1][1] - rs[i+1][1]) < para.cent_dif:
                rdel.append(region.copy().pop(ind[i+1]))
                
        for i in range(len(rdel)):
            region.remove(rdel[i])
            
        # since only 1 object, only 1 region should be identified
        if len(region) > 1:
            for i in range(len(region)):
                print(region[i].centroid, region[i].bbox_area)
            plt.imshow(tu_b, cmap = 'gray')
            fig = plt.get_current_fig_manager()
            fig.window.setGeometry(400, 100, 3000, 2000)
            plt.title('You found more than 1 contour on this image!!!', fontsize = 66)
        else:
            minr, minc, maxr, maxc = region[0].bbox # max/min row/column coordinates
            box_how.append((maxr-minr)/(maxc-minc))
    
    # Store extreme values
    max_ratio = max(box_how)
    min_ratio = min(box_how)       
    
    return max_ratio, min_ratio



def obtain_testing_y(tu, omega, alg):
    """ stores several training algorithms """
              
              
    # Create a binary image
    tu_b = np.zeros((len(tu[:, 0, 0]), len(tu[0, :, 0]))) # initialize the binary image to have the same dimensions
   
    if alg == 'LR':
        for i in range(len(tu[:, 0, 0])): # how many rows
            for j in range(len(tu[0, :, 0])): # how many columns
                if np.dot(tu[i, j, :], omega) >= 0.5: # y(wanted) = 1, y(else) = 0
                    tu_b[i, j] = 1 # wanted = white on gray scale
                else:
                    tu_b[i, j] = 0 # unwanted = black on gray scale
    
    elif alg == 'GNB': # Gaussian Naive Bayes 
        for i in range(len(tu[:, 0, 0])): # how many rows
            for j in range(len(tu[0, :, 0])): # how many columns
                he0 = 0
                he1 = 0
                for k in range(3):
                    he0 += np.log(omega[8 + k]) + (tu[i, j, k] - omega[2 + k]) ** 2 / omega[8 + k]
                    he1 += np.log(omega[11 + k]) + (tu[i, j, k] - omega[5 + k]) ** 2 / omega[11 + k]
                if np.log(1 / omega[0] ** 2) + he0 > np.log(1 / omega[1] ** 2) + he1:
                    tu_b[i, j] = 1 # wanted = white on gray scale
                else:
                    tu_b[i, j] = 0 # unwanted = black on gray scale
        

    return tu_b



