# -*- coding: utf-8 -*-
"""
Created on Mon Apr 22 15:05:35 2019

@author: m8mao
"""

# there should be a better way to handle all these widgets(such as use canvas text instead of label)

import os # used to locate image
import numpy as np # used to save and load
import time # used for animation

import tkinter as tk # GUI
from PIL import Image, ImageTk # load image

import search_algos as sa # search algorithm
from puzzleclass import My8PuzzleStates # treat tiles as objects


class PuzzleGUI():
    """ The 8-puzzle board object """
    
    
    def __init__(self, root, geo):
        
        
        self.root = root
        self.side = geo[0]
        self.offw = geo[1]
        self.offh = geo[2]
        self.c = tk.Canvas(root, width = geo[3], height = geo[4], bg = 'white')
        self.c.pack() # make canvas fit
        self.e = []
        for i in range(9):
            self.e.append(tk.Entry(root))
        self.v = tk.StringVar()
        current_dir = os.getcwd() + '\images\mylogo.png'
        logo = ImageTk.PhotoImage(Image.open(current_dir).resize((int(geo[3]/10), int(geo[4]/10))), 
                                  master = root) # no need to have master if don't resize
        self.c.copy_image = logo # use copy_image so that image always stays
        self.c.create_image(7*self.side + self.offw, 4.5*self.side + self.offh, image = self.c.copy_image)
        
        # Buttons
        self.b1 = tk.Button(root)
        self.b1.config(text = 'Set Board', command = self.initialize, width = 8)
        self.b1.place(x = self.offw, y = 3.2*self.side + self.offh)
        
        self.b2 = tk.Button(root)
        self.b2.config(text = 'Start', command = self.update, width = 8)
        self.b2.place(x = self.offw + 1.05*self.side, y = 3.2*self.side + self.offh)
        
        self.b3 = tk.Button(root)
        self.b3.config(text = 'Reset', command = self.reset, width = 8)
        self.b3.place(x = self.offw + 2.1*self.side, y = 3.2*self.side + self.offh)
        
        self.bBFS = tk.Button(root)
        self.bBFS.config(text = 'BFS', command = lambda s = 'BFS': self.algorithm(s), width = 8)
        self.bBFS.place(x = self.offw + 3.1*self.side, y = self.side/4 + self.offh)
        
        self.bDFS = tk.Button(root)
        self.bDFS.config(text = 'DFS', command = lambda s = 'DFS': self.algorithm(s), width = 8)
        self.bDFS.place(x = self.offw + 3.1*self.side, y = self.side*5/4 + self.offh)
        
        self.bAST = tk.Button(root)
        self.bAST.config(text = 'A*', command = lambda s = 'A*': self.algorithm(s), width = 8)
        self.bAST.place(x = self.offw + 3.1*self.side, y = self.side*9/4 + self.offh)
        
        # Texts
        self.tINFO = tk.Label(root, 
                           text = '1. Type down numbers in the left board \n2. Press "Set Board" to initialize board \n3. Select an algorithm \n4. Press "Start" to see solution') # cannot break lines here so let's make it even longer
        self.tINFO.config(font = ('Courier', 9, 'bold'), bg = 'white', 
                       anchor = 'w', justify = 'left')
        self.tINFO.place(x = self.offw, y = 4*self.side + self.offh)
        
        self.tERROR = tk.Label(root) # error message
        
        self.tN1 = tk.Label(root, text = '# of Nodes Expanded', 
                            font = ('Courier', 9, 'bold'), bg = 'white', fg = 'deep sky blue')    
        self.tN1.place(x = self.offw, y = 5.4*self.side + self.offh)  
        self.tN2 = tk.Label(root, font = ('Courier', 9, 'bold'), bg = 'white', fg = 'deep sky blue')    
        self.tN2.place(x = 1.5*self.side + self.offw, y = 5.7*self.side + self.offh) 
        
        self.tCOST1 = tk.Label(root, text = 'Depth or Cost',
                            font = ('Courier', 9, 'bold'), bg = 'white', fg = 'deep sky blue')    
        self.tCOST1.place(x = 3*self.side + self.offw, y = 5.4*self.side + self.offh) 
        self.tCOST2 = tk.Label(root, font = ('Courier', 9, 'bold'), bg = 'white', fg = 'deep sky blue')
        self.tCOST2.place(x = 3.7*self.side + self.offw, y = 5.7*self.side + self.offh)  
        
        self.tTIME1 = tk.Label(root, text = 'Time (s)',
                            font = ('Courier', 9, 'bold'), bg = 'white', fg = 'deep sky blue')  
        self.tTIME1.place(x = 5*self.side + self.offw, y = 5.4*self.side + self.offh) 
        self.tTIME2 = tk.Label(root, font = ('Courier', 9, 'bold'), bg = 'white', fg = 'deep sky blue')  
        self.tTIME2.place(x = 5*self.side + self.offw, y = 5.7*self.side + self.offh)          
        
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\#        
    
    def algorithm(self, name):
        """ select which algorithm to use based on which button is pressed """


        if name == 'BFS':
            self.v = 'BFS'
            self.bBFS.config(relief = 'sunken')
            self.bDFS.config(relief = 'raised')
            self.bAST.config(relief = 'raised')
            
        if name == 'DFS':
            self.v = 'DFS'
            self.bDFS.config(relief = 'sunken')
            self.bBFS.config(relief = 'raised')
            self.bAST.config(relief = 'raised')
            
        if name == 'A*':
            self.v = 'A*'
            self.bAST.config(relief = 'sunken')
            self.bBFS.config(relief = 'raised')
            self.bDFS.config(relief = 'raised')
        
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\#        
        
    def draw_tiles(self, x0_s):
        """ draw empty tiles in grid form """
        
        
        for i in range(3): # row 0
            self.c.create_rectangle(x0_s + self.side*i, self.offh, # top-left corner
                                    x0_s + self.side*(i+1), self.side + self.offh, # bottom-right corner
                                    fill = 'darkseagreen1', width = 5) 
            
        for i in range(3): # row 1
            self.c.create_rectangle(x0_s + self.side*i, self.side + self.offh, 
                                    x0_s + self.side*(i+1), self.side*2 + self.offh, 
                                    fill = 'darkseagreen1', width = 5)
            
        for i in range(3): # row 2
            self.c.create_rectangle(x0_s + self.side*i, self.side*2 + self.offh, 
                                    x0_s + self.side*(i+1), self.side*3 + self.offh, 
                                    fill = 'darkseagreen1', width = 5)
            
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\#              
            
    def initialize(self):
        """ process user input to initialize the puzzle board """
    
        
        # Read input
        z = [] # a dummy variable
        state = [] # store values in numerical form
        state_check = [] # used to check if solvable
        for i in range(9):
            z.append(self.e[i].get()) 
            if z[i] != '':
                state.append(int(z[i]))
                state_check.append(int(z[i]))
            else:
                pos0 = i
                state.append(0)
        r0, r1, r2 = z[0:3], z[3:6], z[6:9] # separated by row
        
        ind = sorted(range(len(state_check)), key = state_check.__getitem__)
        inversions = 0
        for i in range(1, len(state_check)):
            for x in ind[:i]:
                if x > ind[i]:
                    inversions += 1  
           
        # Check input
        good = True
        base = [str(x) for x in list(range(1, 9, 1))]
        
        check = z[:]
        del check[pos0]
        seen = []
        for i in range(len(check)):
            if (check[i] in base) and (check[i] not in seen): # good input
                if (inversions % 2 == 0):
                    seen.append(check[i])
                else:
                    self.tERROR.config(text = 'Insolvable input, please try something else!', 
                                   font = ('Courier', 9, 'bold'), fg = 'tomato', bg = 'white')
                    self.tERROR.place(x = self.offw, y = 5*self.side + self.offh)
                    good = False
                    break
            else:
                self.tERROR.config(text = 'Bad input, please try something else!', 
                                   font = ('Courier', 9, 'bold'), fg = 'tomato', bg = 'white')
                self.tERROR.place(x = self.offw, y = 5*self.side + self.offh)
                good = False
                break
                    
        # Board (Tile + Number)
        if good is True:
            self.tERROR.config(text = '') # didn't use destroy() b.c. tERROR no longer exists until restart
            
        x0_s, x0_t = self.side*5 + self.offw, self.side*11/2 + self.offw # coordinate for tile and text  
        for i in range(3): # row 0
            if i != pos0:
                self.c.create_rectangle(x0_s + self.side*i, self.offh, # top-left corner
                                   x0_s + self.side*(i+1), self.side + self.offh, # bottom-right corner
                                   tag = 'c' + str(i), fill = 'darkseagreen1', width = 5) # tag to combine 2 objects
                self.c.create_text(x0_t + self.side*i, self.side/2 + self.offh, text = r0[i], 
                              tag = 'c' + str(i), fill = 'peachpuff3', font = 'Times 20 bold')
            
        for i in range(3): # row 1
            if i != pos0 - 3:
                self.c.create_rectangle(x0_s + self.side*i, self.side + self.offh, 
                                   x0_s + self.side*(i+1), self.side*2 + self.offh, 
                                   tag = 'c' + str(i+3), fill = 'darkseagreen1', width = 5)
                self.c.create_text(x0_t + self.side*i, self.side*3/2 + self.offh, text = r1[i], 
                              tag = 'c' + str(i+3), fill = 'peachpuff3', font = 'Times 20 bold')
            
        for i in range(3): # row 2
            if i != pos0 - 6:
                self.c.create_rectangle(x0_s + self.side*i, self.side*2 + self.offh, 
                                   x0_s + self.side*(i+1), self.side*3 + self.offh, 
                                   tag = 'c' + str(i+6), fill = 'darkseagreen1', width = 5)
                self.c.create_text(x0_t + self.side*i, self.side*5/2 + self.offh, text = r2[i], 
                              tag = 'c' + str(i+6), fill = 'peachpuff3', font = 'Times 20 bold')
            
        np.save('Your Puzzle', state)    
    
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\#      
    
    def reset(self):
        """ clear input board and reset output board """
        
        
        # Clear input board
        for i in range(9):
            self.e[i].delete(0, 'end')
            
        # Clear output board
        self.draw_tiles(self.side*5 + self.offw)
        
        # Clear messages
        self.tERROR.config(text = '')
        
        # Clear algorithm
        self.v = '0'
        self.bBFS.config(relief = 'raised')
        self.bDFS.config(relief = 'raised')
        self.bAST.config(relief = 'raised')
        
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\#           
            
    def propose__puzzle(self):
        """ request user to propose a puzzle by typing numbers and trigger """
         

        # Tiles
        self.draw_tiles(self.offw)
    
        # Numbers
        state = []
        for i in range(3):
            self.e[i].place(width = 80, x = self.side*i + self.side/2, 
                       y = self.side/2 - self.offh*2) # strange offsets make cursor at center   
            self.e[i].config(bg = 'darkseagreen1', font = 'Times 20 bold', 
                        borderwidth = 0, insertwidth = 10) # only show cursor 
            state.append(self.e[i])
            
        for i in range(3):
            self.e[i+3].place(width = 80, x = self.side*i + self.side/2, 
                       y = self.side*3/2 - self.offh*2) # strange offsets make cursor at center   
            self.e[i+3].config(bg = 'darkseagreen1', font = 'Times 20 bold', 
                        borderwidth = 0, insertwidth = 10) # only show cursor 
            state.append(self.e[i+3])
            
        for i in range(3):
            self.e[i+6].place(width = 80, x = self.side*i + self.side/2, 
                       y = self.side*5/2 - self.offh*2) # strange offsets make cursor at center   
            self.e[i+6].config(bg = 'darkseagreen1', font = 'Times 20 bold', 
                        borderwidth = 0, insertwidth = 10) # only show cursor 
            state.append(self.e[i+6])
        
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\#           
        
    def update(self):
        """ swap tiles based on calculated moves (animation of solution) """
        
        
        # Solve the puzzle
        if self.v == 'BFS':
            z = sa.BFS(My8PuzzleStates(list(np.load('Your Puzzle.npy')))) 
        elif self.v == 'DFS':
            z = sa.DFS(My8PuzzleStates(list(np.load('Your Puzzle.npy')))) 
        elif self.v == 'A*':
            z = sa.Astar(My8PuzzleStates(list(np.load('Your Puzzle.npy'))))
        else:
            self.tERROR.config(text = 'A* was selected as default',
                           font = ('Courier', 9, 'bold'), fg = 'tomato', bg = 'white')
            self.tERROR.place(x = self.offw, y = 5*self.side + self.offh)
            z = sa.Astar(My8PuzzleStates(list(np.load('Your Puzzle.npy'))))
        
        # Allocate useful info
        solution = sa.backtracking(z)  
        state = [x[0] for x in solution]
        moves = [x[1] for x in solution]
        costs = [x[2] for x in solution]
        nnode = [x[-2] for x in solution]
        times = [x[-1] for x in solution]
        pos0 = state[0].index(0)
        
        for i in range(len(moves)):
            
            # Update tiles
            if moves[i] == 'START':
                pass
            if moves[i] == 'U':
                tagp = state[0].index(state[i-1][pos0 + 3])
                self.c.move('c' + str(tagp), 0, -self.side)
            if moves[i] == 'D':
                tagp = state[0].index(state[i-1][pos0 - 3])
                self.c.move('c' + str(tagp), 0, self.side)   
            if moves[i] == 'L':
                tagp = state[0].index(state[i-1][pos0 + 1])
                self.c.move('c' + str(tagp), -self.side, 0)
            if moves[i] == 'R':
                tagp = state[0].index(state[i-1][pos0 - 1])
                self.c.move('c' + str(tagp), self.side, 0)
            
            # Update # of nodes
            self.tN2.config(text = str(nnode[i]))
            
            # Update depth(cost)
            self.tCOST2.config(text = str(costs[i]))
            
            # Update time
            self.tTIME2.config(text = str(times[i]))
            
            pos0 = state[i].index(0)
            self.root.update()
            time.sleep(0.5)
            
            
            



