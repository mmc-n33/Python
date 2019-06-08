# -*- coding: utf-8 -*-
"""
Created on Mon Apr 22 09:34:42 2019

@author: m8mao
"""

import time # used for time recording
from search_algos import Manhattan # import Manhattan


class My8PuzzleStates():
    """ This class stores the 8-puzzle states object as a list of size of 9 """
    
    
    def __init__(self, s_init, move = 'START', depth = 0):

        
        self.pos0 = [] # 0 represents the blank tile
        self.nn = 1 # number of node expanded
        # self.parent[0] = current nodes state,                     such as [0,1,2,3,5,4,6,7,8]
        # self.parent[1] = action leads to current node,            such as 'U'
        # self.parent[2] = current node's depth,                    such as 20
        # self.parent[3] = current node's distance to goal node,    such as 16
        # self.parent[4] = up till now, # of nodes expanded,        such as 666
        # self.parent[5] = seconds took from last node to current,  such as 0.0004
        self.parent = [s_init, move, depth, Manhattan(s_init), self.nn, 0] 
        self.children = []
        self.tic = time.time()



    def expand(self):
        """ given a node, return all its childern nodes """
            
        
        self.pos0 = self.parent[0].index(0)
        self.children = []
        
        self.move__up()
        self.move__down()
        self.move__left()
        self.move__right()

        return self.children
            


    def move__up(self):
        """ move one tile up to swap position with the blank tile """
        
        
        temp = self.parent[0][:] # if copy the whole self.parent, somehow [:] is no longer shallow
        if self.pos0 in [6, 7, 8]:
            pass
        else:
            temp[self.pos0], temp[self.pos0 + 3] = \
            self.parent[0][self.pos0 + 3], self.parent[0][self.pos0]
            self.nn += 1 # see one more node
            
            self.children.append([temp, 'U', self.parent[2] + 1, Manhattan(temp), 
                                  self.nn, float(time.time() - self.tic)])
    
    
    
    def move__down(self):
        """ move one tile down to swap position with the blank tile """
           
        
        temp = self.parent[0][:]
        if self.pos0 in [0, 1, 2]:
            pass
        else:
            temp[self.pos0], temp[self.pos0 - 3] = \
            self.parent[0][self.pos0 - 3], self.parent[0][self.pos0]
            self.nn += 1 
            
            self.children.append([temp, 'D', self.parent[2] + 1, Manhattan(temp), 
                                  self.nn, float(time.time() - self.tic)])
    
    
    
    def move__left(self):
        """ move one tile left to swap position with the blank tile """
         
        
        temp = self.parent[0][:]
        if self.pos0 in [2, 5, 8]:
            pass
        else:
            temp[self.pos0], temp[self.pos0 + 1] = \
            self.parent[0][self.pos0 + 1], self.parent[0][self.pos0]
            self.nn += 1

            self.children.append([temp, 'L', self.parent[2] + 1, Manhattan(temp), 
                                  self.nn, float(time.time() - self.tic)])
    
    
    
    def move__right(self):
        """ move one tile right to swap position with the blank tile """
        
        
        temp = self.parent[0][:]
        if self.pos0 in [0, 3, 6]:
            pass
        else:
            temp[self.pos0], temp[self.pos0 - 1] = \
            self.parent[0][self.pos0 - 1], self.parent[0][self.pos0]
            self.nn += 1
            
            self.children.append([temp, 'R', self.parent[2] + 1, Manhattan(temp), 
                                  self.nn, float(time.time() - self.tic)])
            
          
            
