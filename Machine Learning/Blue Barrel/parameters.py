# -*- coding: utf-8 -*-
"""
Created on Tue Jan 29 15:44:59 2019

@author: Mingchen Mao
"""


class Parameters():
    """ Store parameters that may be useful in barrel detection """
    
    def __init__(self):
        
        self.bbox_area = 666 # any bounding box with an area < "bbox_area" will be removed
        self.cent_dif = 20 # any 2 bounding boxes with centroids difference < "cent_dif", one of them will be removed
        self.step_size = 0.1 # step size for iterations
        self.iteration = 10 # times of iterations
        self.max_ratio = 2.5
        self.min_ratio = 1.35
        