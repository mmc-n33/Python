# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 17:00:25 2019

@author: Mingchen Mao
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation



def plot__animatedly(para, fig, ax, im, point, text, lsr, ltr, lsw, ltw, data, pointdata, textdata, interval = 1):
    """ create animated plot for the robot and its discovery, including a time count
    
    Inputs:
        para = associated parameters in the Parameters class
        fig = figure window
        ax = axis of figure
        im = image object which will be modified in this animation
        point = points object which will be modified in this animation
        text = text object which will be modified in this animation
        lsr = robot legend symbol object which will be unchanged in this animation
        ltr = robot legend text object which will be unchanged in this animation
        lsw = wall legend symbol object which will be unchanged in this animation
        ltw = wall legend text object which will be unchanged in this animation
        data = data used to modify 'im' (map data at each time step)
        pointdata = data used to modify 'point' (best particle pose at each time step)
        textdata = data used to modify 'text' (starts at 0 s with time step of 0.025 s)
        interval = control how fast new frame is blit in the animation plot (1 = 0.001 s) 
    Outputs:
        gif = an animation object """
    
    
    def init():
        """ create the base frame of the animation plot
        
        Output:
            im, = a tuple of plot objects to be updated """
            
            
        im.set_data(np.empty((para.cell_num, para.cell_num)))
        point.set_data([], [])
        text.set_text('')
        x = lsr.get_xdata() 
        y = lsr.get_ydata()
        lsr.set_data(x, y)
        ltr.set_text('')
        x = lsw.get_xdata()
        y = lsw.get_ydata()
        lsw.set_data(x, y)
        ltw.set_text('')
        
        return im,
    
    
    
    def animate(i):
        """ update the animation plot at each frame """
            
                
        im.set_array(data[i]) 
        point.set_data(pointdata[0 : i, 0], pointdata[0 : i, 1])
        text.set_text('Time = %.2f s' % textdata[i])
        x = lsr.get_xdata() 
        y = lsr.get_ydata()
        lsr.set_data(x, y)
        ltr.set_text('Predicted Robot Position')
        x = lsw.get_xdata()
        y = lsw.get_ydata()
        lsw.set_data(x, y)
        ltw.set_text('Predicted Wall Position')
        
        return im, point, text, lsr, ltr, lsw, ltw,
    
    # save_count = how many frames to save when gif is called to be saved as mp4
    # blit = only update the parts that has been changed (seems useless in this case)
    gif = animation.FuncAnimation(fig, animate, init_func = init, interval = interval, 
                                  save_count = len(data), blit = True)
    
    return gif