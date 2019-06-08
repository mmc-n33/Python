# -*- coding: utf-8 -*-
"""
Created on Fri Apr 19 23:00:46 2019

@author: m8mao
"""

# seems always expand some extra nodes? 
# time-wise performance may be improved by playing with code structure (object?) and data structure (numpy?)

import numpy as np # used by Manhattan


def BFS(node):
    """ Breadth-First Search """
        
    
    fringe = [node.parent] # waiting room
    explored = []

    while True:
        
        node.parent = fringe.pop(0)
        print(node.parent)
        explored.append(node.parent)
 
        if node.parent[0] == list(range(0, 9, 1)):
            break
        
        neighbors = node.expand()
        for i in range(len(neighbors)):
            if (neighbors[i][0] not in [x[0] for x in fringe]) and \
                (neighbors[i][0] not in [x[0] for x in explored]):
                fringe.append(neighbors[i]) # add new node to the end of queue
                     
    return explored



def DFS(node):
    """ Depth-First Search """
    
    
    fringe = [node.parent] # waiting room
    explored = []

    while True:
        
        node.parent = fringe.pop(0)
        explored.append(node.parent)

        if node.parent[0] == list(range(0, 9, 1)):
            break
        
        neighbors = node.expand()
        j = 0
        for i in range(len(neighbors)):
            if (neighbors[i][0] not in [x[0] for x in fringe]) and \
                (neighbors[i][0] not in [x[0] for x in explored]):
                fringe.insert(j, neighbors[i]) # push new node into stack
                j += 1
        
    return explored  
        
        
        
def Astar(node):
    """ A* Search """
    
    
    fringe = [node.parent] # waiting room
    explored = []
    
    
    while True:
        
        fringe = sorted(fringe, key=lambda x: x[2] + x[3]) # prioritize expanded nodes
        node.parent = fringe.pop(0)
        explored.append(node.parent)

        if node.parent[0] == list(range(0, 9, 1)):
            break
        
        neighbors = node.expand()
        for i in range(len(neighbors)):
            if (neighbors[i][0] not in [x[0] for x in fringe]) and \
                (neighbors[i][0] not in [x[0] for x in explored]):
                fringe.append(neighbors[i])
        
    return explored



def backtracking(config):
    """ given explored history, extract useful moves that lead from IC to success """

  
    config = list(reversed(config)) # reverse order so that easier to "bookkeep"
    rightway = [0]
    i = 0 
    
    while True:
    
        temp = config[i][0][:] # shallow copy
        pos0 = temp.index(0)
        
        if config[i][1] == 'U':
            temp[pos0], temp[pos0 - 3] = config[i][0][pos0 - 3], config[i][0][pos0]
            i = [x[0] for x in config].index(temp)
            if i == len(config) - 1: # reach the end of list, namely IC
                break
            rightway.append(i)
            
        elif config[i][1] == 'D':
            temp[pos0], temp[pos0 + 3] = config[i][0][pos0 + 3], config[i][0][pos0]
            i = [x[0] for x in config].index(temp)
            if i == len(config) - 1:
                break
            rightway.append(i)
            
        elif config[i][1] == 'L':
            temp[pos0], temp[pos0 - 1] = config[i][0][pos0 - 1], config[i][0][pos0]
            i = [x[0] for x in config].index(temp)
            if i == len(config) - 1:
                break
            rightway.append(i)
            
        elif config[i][1] == 'R':
            temp[pos0], temp[pos0 + 1] = config[i][0][pos0 + 1], config[i][0][pos0]
            i = [x[0] for x in config].index(temp)
            if i == len(config) - 1:
                break
            rightway.append(i)
            
    rightway.append(len(config)-1) # add IC
        
    return list(reversed([config[j] for j in rightway]))



def Manhattan(s_c):
    """ calculate a heuristic to measure the "distance" between current puzzle states
        and the correct puzzle states, using Mantattan priority function """
        

    r_goal = np.array([0, 0, 1, 1, 1, 2, 2, 2]) # goal row
    c_goal = np.array([1, 2, 0, 1, 2, 0, 1, 2]) # goal column
    
    indices = np.array(sorted(range(len(s_c)), key = s_c.__getitem__)) # sort current state in increasing order
    indices = np.delete(indices, 0) # blank tile will not be counted
    
    r_c = np.floor(indices / 3) # current row
    c_c = np.remainder(indices, 3) # current column
    
    heuristic = np.sum(abs(r_goal - r_c)) + np.sum(abs(c_goal - c_c))
    
    return heuristic


#%% test
#from PuzzleClass import My8PuzzleStates
##s_init = [3,1,2,6,4,5,0,7,8]
#s_init = [6,1,8,4,0,2,7,3,5]
##s_init = [3,0,2,4,1,7,6,8,5]
###s_init = [3,0,2,6,4,5,1,7,8]
#m8ps = My8PuzzleStates(s_init)
#z = BFS(m8ps) 
##z=Astar(m8ps)
#solution= backtracking(z)


