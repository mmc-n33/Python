# -*- coding: utf-8 -*-
"""
Created on Tue Apr 23 14:56:19 2019

@author: m8mao
"""

  
import os
import tkinter as tk # GUI

from puzzlegui import PuzzleGUI

  
if os.path.exists('Your Puzzle.npy') == True:
    os.remove('Your Puzzle.npy')
    
root = tk.Tk() # create the root widget
root.title("Mao's 8 Puzzle Solver")

side = 200 # tile side length
offw, offh = 0.1*side, 0.06*side # chess board edges have these offsets to canvas edges
w, h = int(8*side + 2*offw), int(6*side + 2*offh) # window(canvas) width and height

geo = [side, offw, offh, w, h]
PuzzleGUI(root, geo).propose__puzzle()

root.geometry(str(w) + 'x' + str(h))
root.mainloop() # enter the Tkinter event loop to show window