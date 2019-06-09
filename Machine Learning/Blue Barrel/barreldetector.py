# -*- coding: utf-8 -*-
"""
Created on Tue Jan 29 15:44:59 2019

@author: Mingchen Mao
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import image as img
from skimage.measure import label, regionprops
import trainingfunctions as tf


class BarrelDetector():
    def __init__(self, para, tu):
        """ assign training parameters and load images from the testset folder """
        
        
        self.para = para
        self.tu = tu  # the image
        

    def segment_image(self, *loadname, alg = 'LR', tbt = False, cmap = None, display = True):
        """ convert an image into binary format, namely the color(s) we want and other colors 

        INPUT: 
            tbt = train bu train, if True, train a new model, otherwise load last training results
            display = if True, display the binary image, otherwise only return tu_b """
         

        # Preliminaries
        alg_list = ['LR', 'GNB', 'GM']
        if alg not in alg_list:
            print('The algorithm you request has not been coded yet, it has been set to logistic regression')
            alg = 'LR'
        if len(loadname) > 0: # if loadname is input
            loadname = loadname[0]
            
        # Train
        if tbt: # if train a new model
            xt = np.load(loadname + '.npy')
            xt = xt.item() # numpy uses item instead of items
            x = np.zeros((1, 3))
            y = []
    
            # Process training data into a 3xd matricx with corresponding class values
            for k,v in xt.items():
                print(k)
                if k[:2] == 'bu': # unwanted color
                    y = np.append(y, np.zeros((len(v), 1))) # y(wanted) = 1, y(else) = 0
                else:
                    y = np.append(y, np.ones((len(v), 1)))
                x = np.append(x, v, axis = 0)
            x = x[1:] # get rid of the pre-assigned row
        
            # Iterate to find the optimal parameters
            omega = tf.obtain_training_parameters(self.para, x, y, alg)
            np.save('omega' + alg, omega)
            
        else: 
            if display:
                omega = np.load('omega' + alg + '.npy')
                
                # Test
                tu_b = tf.obtain_testing_y(self.tu, omega, alg)
                
                # Plot
                plt.figure()
                plt.imshow(tu_b, cmap) 
                fig = plt.get_current_fig_manager()
                fig.window.setGeometry(400, 100, 3000, 2000) # display figure properly on 4K resolution
        
                return tu_b


    def get_bounding_box(self, tu_b):
        """ identify blue barrels on given images by plotting bounding boxes and return their coordinates
            this function requires the output from the "segment_image" function """
    
    
        tu_b = tu_b.astype(np.uint8) # convert binary image to a format that @findContours can process
        global box
        global no_det
        
        # Find contours of objects with wanted color
        contours, hierachy = cv2.findContours(tu_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(tu_b, contours, -1, (255, 0, 0), 3) # replace binary image by its binary contour image, -1 = draw all contours, (color), thickness of contour lines

        # Get contours edges, namely bounding boxes
        tu_b = label(tu_b) # label connected regions of an integer array, so that unconnected contours will considered as separate regions
        region = regionprops(tu_b) # identify regions of the labeled image
        rc = [] # region's centroids

        # Get rid of tiny regions
        for prop in region.copy():
            if prop.bbox_area < self.para.bbox_area:
                region.remove(prop)
           
        # Get rid of un-barrel shapes
        for prop in region.copy():
            minr, minc, maxr, maxc = prop.bbox # max/min row/column coordinates
            if (maxr-minr)/(maxc-minc) > self.para.max_ratio or (maxr-minr)/(maxc-minc) < self.para.min_ratio:
                print('Height/Width = ' + str((maxr-minr)/(maxc-minc)) + 'has been eliminated')
                region.remove(prop)
            else:
                rc.append(prop.centroid)
                
        # Get rid of repeated regions
        ind = sorted(range(len(rc)), key = rc.__getitem__) # store element indices of local_centroid tuples before sorting
        rs = sorted(rc)

        rdel = [] # repeated regions to be deleted
        for i in range(0, len(rs) - 1):
            if abs(rs[i+1][0] - rs[i][0]) < self.para.cent_dif and abs(rs[i+1][1] - rs[i+1][1]) < self.para.cent_dif:
                rdel.append(region.copy().pop(ind[i+1]))
                
        for i in range(len(rdel)):
            region.remove(rdel[i])
          
        # Call the original image
        plt.figure()
        plt.imshow(self.tu) # the orginal image
        f = plt.get_current_fig_manager()
        f.window.setGeometry(400, 100, 3000, 2000) # display figure properly on 4K resolution
        
        # Get valid bounding box coordinates and plot on the original image
        if not len(region): # if no blue barrel was detected
            box = [0, 0, 0, 0]
            no_det = 1 # no box was detected
        else:
            no_det = 0 # if detected, draw all boxes
            for prop in region:
                minr, minc, maxr, maxc = prop.bbox # max/min row/column coordinates
                box = [minc, minr, maxc, maxr] # [x1, y1, x2, y2], where (x1, y1) and (x2, y2) are the top left and bottom right coordinate respectively
                bx = (minc, maxc, maxc, minc, minc) # corner coordinates - x
                by = (minr, minr, maxr, maxr, minr) # corner coordinates - y
                plt.plot(bx, by, '-r', linewidth = 3)
                #plt.show()
       
        return box, no_det

    


